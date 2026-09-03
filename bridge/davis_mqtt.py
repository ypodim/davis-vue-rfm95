#!/usr/bin/env python3
"""Bridge the Davis receiver's serial stream to MQTT for Home Assistant.

Reads the JSON-per-line output of firmware/davis-track.ino and republishes it
as Home Assistant sensors via MQTT auto-discovery.

Deliberately stores nothing on disk. All state is in memory, and Home
Assistant's own recorder keeps the history -- this host is a pass-through.
A restart simply re-baselines and carries on.

Why a single retained state topic rather than one topic per sensor: the ISS
rotates through sensor types, so each transmission carries wind plus exactly
one other measurement. Publishing a merged JSON document means Home Assistant
always sees a complete picture instead of sensors that individually go stale
between their turns in the rotation.

Usage:
    python3 davis_mqtt.py --broker 192.168.1.100 --user mqtt_user --password ...

Credentials can also come from the environment (MQTT_USER / MQTT_PASSWORD),
which is what the systemd unit uses so they stay out of the process list.
"""

import argparse
import json
import logging
import os
import signal
import sys
import time

import paho.mqtt.client as mqtt
import serial

log = logging.getLogger("davis-mqtt")

NODE = "davis_vue"
DISCOVERY_PREFIX = "homeassistant"
STATE_TOPIC = f"davis/{NODE}/state"
AVAIL_TOPIC = f"davis/{NODE}/availability"

DEVICE = {
    "identifiers": [NODE],
    "name": "Davis Vantage Vue",
    "manufacturer": "Davis Instruments",
    "model": "Vantage Vue ISS 6357 (via Feather RP2040 RFM95)",
}

# key in state JSON -> (friendly name, unit, device_class, state_class, icon)
# Only sensors this station actually reports. The Vue ISS has no UV or solar
# sensor -- those are Vantage Pro2 options -- so they are absent by design.
SENSORS = {
    "temperature": ("Temperature", "°F", "temperature", "measurement", None),
    "humidity":    ("Humidity", "%", "humidity", "measurement", None),
    "wind_speed":  ("Wind Speed", "mph", "wind_speed", "measurement", None),
    "wind_dir":    ("Wind Direction", "°", None, "measurement", "mdi:compass-outline"),
    "rain_total":  ("Rain Since Start", "in", "precipitation", "total_increasing", None),
    "rssi":        ("Signal Strength", "dBm", "signal_strength", "measurement", None),
    "packet_rate": ("Packet Rate", "/min", None, "measurement", "mdi:radio-tower"),
}

BINARY_SENSORS = {
    "battery_low": ("Battery Low", "battery"),
}


class Bridge:
    def __init__(self, args):
        self.args = args
        self.state = {}
        self.last_rain_count = None
        self.rain_total = 0.0
        self.packet_times = []
        self.running = True

        self.client = mqtt.Client(client_id=f"{NODE}-bridge")
        if args.user:
            self.client.username_pw_set(args.user, args.password)
        self.client.will_set(AVAIL_TOPIC, "offline", retain=True)
        self.client.on_connect = self._on_connect

    def _on_connect(self, client, _u, _f, rc):
        if rc != 0:
            log.error("MQTT connect failed rc=%s", rc)
            return
        log.info("connected to %s:%s", self.args.broker, self.args.port)
        self._publish_discovery()
        client.publish(AVAIL_TOPIC, "online", retain=True)

    def _publish_discovery(self):
        for key, (name, unit, dev_cls, state_cls, icon) in SENSORS.items():
            cfg = {
                "name": name,
                "unique_id": f"{NODE}_{key}",
                "state_topic": STATE_TOPIC,
                "value_template": "{{ value_json.%s }}" % key,
                "availability_topic": AVAIL_TOPIC,
                "device": DEVICE,
            }
            if unit:
                cfg["unit_of_measurement"] = unit
            if dev_cls:
                cfg["device_class"] = dev_cls
            if state_cls:
                cfg["state_class"] = state_cls
            if icon:
                cfg["icon"] = icon
            # Diagnostics belong in HA's diagnostic section, not the main card.
            if key in ("rssi", "packet_rate"):
                cfg["entity_category"] = "diagnostic"
            self.client.publish(
                f"{DISCOVERY_PREFIX}/sensor/{NODE}/{key}/config",
                json.dumps(cfg), retain=True)

        for key, (name, dev_cls) in BINARY_SENSORS.items():
            cfg = {
                "name": name,
                "unique_id": f"{NODE}_{key}",
                "state_topic": STATE_TOPIC,
                "value_template": "{{ 'ON' if value_json.%s else 'OFF' }}" % key,
                "availability_topic": AVAIL_TOPIC,
                "device_class": dev_cls,
                "entity_category": "diagnostic",
                "device": DEVICE,
            }
            self.client.publish(
                f"{DISCOVERY_PREFIX}/binary_sensor/{NODE}/{key}/config",
                json.dumps(cfg), retain=True)
        log.info("published discovery for %d sensors", len(SENSORS) + len(BINARY_SENSORS))

    def _rain(self, count):
        """Accumulate rainfall from the free-running tip counter.

        The counter wraps at 256. The first reading only sets a baseline: we
        cannot know how much fell before we started listening.

        Note the tipping bucket also counts physical handling of the station,
        so a jump right after someone touches it is not rain. Large jumps are
        rejected rather than published.
        """
        if self.last_rain_count is None:
            self.last_rain_count = count
            return
        delta = (count - self.last_rain_count) % 256
        self.last_rain_count = count
        if delta == 0:
            return
        if delta > 20:
            log.warning("implausible rain delta %d tips, ignoring", delta)
            return
        self.rain_total += delta * self.args.rain_bucket

    def _handle(self, d):
        now = time.time()
        self.packet_times = [t for t in self.packet_times if now - t < 60]
        self.packet_times.append(now)

        if "windSpeedMph" in d:
            self.state["wind_speed"] = d["windSpeedMph"]
        if "windDirDeg" in d:
            self.state["wind_dir"] = d["windDirDeg"]
        if "tempF" in d:
            self.state["temperature"] = d["tempF"]
        if "humidity" in d:
            self.state["humidity"] = d["humidity"]
        if "rainCount" in d:
            self._rain(int(d["rainCount"]))
            self.state["rain_total"] = round(self.rain_total, 3)
        if "rssi" in d:
            self.state["rssi"] = d["rssi"]
        if "battLow" in d:
            self.state["battery_low"] = bool(d["battLow"])

        # rainRate_raw, gust_raw and supercap_raw are intentionally not
        # published: their scaling was never verified, and an unverified
        # number displayed as fact is worse than an absent one.
        self.state["packet_rate"] = len(self.packet_times)
        self.client.publish(STATE_TOPIC, json.dumps(self.state), retain=True)

    def run(self):
        self.client.connect(self.args.broker, self.args.port, keepalive=60)
        self.client.loop_start()

        ser = None
        buf = b""
        while self.running:
            try:
                if ser is None:
                    ser = serial.Serial(self.args.port_dev, 115200, timeout=1)
                    log.info("opened %s", self.args.port_dev)
                chunk = ser.read(256)
            except serial.SerialException as e:
                log.error("serial error: %s -- reopening in 5s", e)
                try:
                    if ser:
                        ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(5)
                continue

            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", "replace").strip()
                i = text.find("{")
                if i < 0:
                    continue          # '#' status lines from the firmware
                try:
                    self._handle(json.loads(text[i:]))
                except ValueError:
                    continue

        self.client.publish(AVAIL_TOPIC, "offline", retain=True)
        self.client.loop_stop()
        self.client.disconnect()

    def stop(self, *_):
        log.info("shutting down")
        self.running = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", default="192.168.1.100")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--user", default=os.environ.get("MQTT_USER"))
    ap.add_argument("--password", default=os.environ.get("MQTT_PASSWORD"))
    ap.add_argument("--port-dev", default="/dev/ttyACM0")
    ap.add_argument("--rain-bucket", type=float, default=0.01,
                    help="inches per bucket tip (0.01 US, 0.007874 metric)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s")

    bridge = Bridge(args)
    signal.signal(signal.SIGINT, bridge.stop)
    signal.signal(signal.SIGTERM, bridge.stop)
    try:
        bridge.run()
    except Exception as e:
        log.error("fatal: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()
