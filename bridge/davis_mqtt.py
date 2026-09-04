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

Availability is tied to the *data*, not the process: if no packet arrives for
--stale-after seconds the device is marked offline, so a Feather that has
been unplugged (or a station that has died) shows as unavailable in HA rather
than frozen at its last reading. The MQTT last-will covers the process itself.

Usage, local broker:
    python3 davis_mqtt.py --broker 192.168.1.100 --user mqtt_user --password ...

Usage, remote broker over TLS with a site-specific identity:
    MQTT_TLS=1 DAVIS_NODE=davis_vue_cabin DAVIS_TOPIC_BASE=cabin/davis \\
    DAVIS_DEVICE_NAME="Davis Vantage Vue (Cabin)" \\
    python3 davis_mqtt.py --broker mqtt.example.org --port 8883

Every option can come from the environment (see main()), which is what the
systemd unit in this directory relies on so that credentials and site names
stay in an env file rather than in the process list or this repository.
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

DISCOVERY_PREFIX = "homeassistant"

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
        self.node = args.node
        self.state_topic = f"{args.topic_base}/state"
        self.avail_topic = f"{args.topic_base}/availability"
        self.device = {
            "identifiers": [self.node],
            "name": args.device_name,
            "manufacturer": "Davis Instruments",
            "model": "Vantage Vue ISS 6357 (via Feather RP2040 RFM95)",
        }

        self.state = {}
        self.last_rain_count = None
        self.rain_total = 0.0
        self.packet_times = []
        self.last_packet = None
        self.online = False
        self.running = True

        self.client = mqtt.Client(client_id=f"{self.node}-bridge")
        if args.user:
            self.client.username_pw_set(args.user, args.password)
        if args.tls:
            # No ca_certs -> the system trust store, which is what a public
            # CA-issued broker certificate needs. --ca overrides for private CAs.
            self.client.tls_set(ca_certs=args.ca)
        self.client.will_set(self.avail_topic, "offline", retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.reconnect_delay_set(min_delay=1, max_delay=60)

    # -- MQTT -----------------------------------------------------------------

    def _on_connect(self, client, _u, _f, rc):
        if rc != 0:
            log.error("MQTT connect failed rc=%s", rc)
            return
        log.info("connected to %s:%s", self.args.broker, self.args.port)
        self._publish_discovery()
        # Whatever we believed before the (re)connect, the broker may hold a
        # stale 'offline' from our last will. Re-assert the truth.
        self._set_online(self.last_packet is not None and
                         time.time() - self.last_packet < self.args.stale_after,
                         force=True)

    def _on_disconnect(self, _c, _u, rc):
        if rc != 0:
            log.warning("MQTT disconnected rc=%s, reconnecting", rc)

    def _set_online(self, online, force=False):
        if online == self.online and not force:
            return
        self.online = online
        self.client.publish(self.avail_topic, "online" if online else "offline",
                            retain=True)
        log.info("availability -> %s", "online" if online else "offline")

    def _publish_discovery(self):
        for key, (name, unit, dev_cls, state_cls, icon) in SENSORS.items():
            cfg = {
                "name": name,
                "unique_id": f"{self.node}_{key}",
                "state_topic": self.state_topic,
                "value_template": "{{ value_json.%s }}" % key,
                "availability_topic": self.avail_topic,
                "device": self.device,
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
                f"{DISCOVERY_PREFIX}/sensor/{self.node}/{key}/config",
                json.dumps(cfg), retain=True)

        for key, (name, dev_cls) in BINARY_SENSORS.items():
            cfg = {
                "name": name,
                "unique_id": f"{self.node}_{key}",
                "state_topic": self.state_topic,
                "value_template": "{{ 'ON' if value_json.%s else 'OFF' }}" % key,
                "availability_topic": self.avail_topic,
                "device_class": dev_cls,
                "entity_category": "diagnostic",
                "device": self.device,
            }
            self.client.publish(
                f"{DISCOVERY_PREFIX}/binary_sensor/{self.node}/{key}/config",
                json.dumps(cfg), retain=True)
        log.info("published discovery for %d sensors", len(SENSORS) + len(BINARY_SENSORS))

    # -- data -----------------------------------------------------------------

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
        self.last_packet = now
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
        self.client.publish(self.state_topic, json.dumps(self.state), retain=True)
        self._set_online(True)

    # -- main loop ------------------------------------------------------------

    def run(self):
        # connect_async + loop_start: the network thread owns connecting and
        # reconnecting, so a broker that is down at boot (or a WAN link that
        # flaps) is retried with backoff instead of killing the process.
        self.client.connect_async(self.args.broker, self.args.port, keepalive=60)
        self.client.loop_start()

        ser = None
        serial_down = False
        buf = b""
        while self.running:
            if self.online and time.time() - self.last_packet > self.args.stale_after:
                log.warning("no packets for %ds", self.args.stale_after)
                self._set_online(False)

            try:
                if ser is None:
                    ser = serial.Serial(self.args.port_dev, 115200, timeout=1)
                    log.info("opened %s", self.args.port_dev)
                chunk = ser.read(256)
            except (serial.SerialException, OSError) as e:
                # Log once per outage, not every 5s: a Feather that is simply
                # not plugged in yet should not flood the journal.
                if not serial_down:
                    log.error("serial error: %s -- retrying every 5s", e)
                    serial_down = True
                try:
                    if ser:
                        ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(5)
                continue
            serial_down = False

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

        self.client.publish(self.avail_topic, "offline", retain=True)
        self.client.loop_stop()
        self.client.disconnect()

    def stop(self, *_):
        log.info("shutting down")
        self.running = False


def main():
    env = os.environ.get
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--broker", default=env("MQTT_HOST", "localhost"))
    ap.add_argument("--port", type=int, default=int(env("MQTT_PORT", "1883")))
    ap.add_argument("--user", default=env("MQTT_USER"))
    ap.add_argument("--password", default=env("MQTT_PASSWORD"))
    ap.add_argument("--tls", action="store_true", default=env("MQTT_TLS", "") == "1",
                    help="connect with TLS (env MQTT_TLS=1)")
    ap.add_argument("--ca", default=env("MQTT_CA"),
                    help="CA bundle for --tls; default is the system trust store")
    ap.add_argument("--node", default=env("DAVIS_NODE", "davis_vue"),
                    help="HA object/unique id prefix; make it unique per site")
    ap.add_argument("--topic-base", default=env("DAVIS_TOPIC_BASE", "davis/davis_vue"),
                    help="state/availability topics live under this")
    ap.add_argument("--device-name", default=env("DAVIS_DEVICE_NAME", "Davis Vantage Vue"))
    ap.add_argument("--port-dev", default=env("DAVIS_SERIAL", "/dev/ttyACM0"))
    ap.add_argument("--rain-bucket", type=float, default=float(env("DAVIS_RAIN_BUCKET", "0.01")),
                    help="inches per bucket tip (0.01 US, 0.007874 metric)")
    ap.add_argument("--stale-after", type=int, default=int(env("DAVIS_STALE_AFTER", "120")),
                    help="seconds without a packet before reporting offline")
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
