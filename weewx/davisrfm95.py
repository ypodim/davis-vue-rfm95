"""weewx driver for a Davis Vantage Vue received via a Feather RP2040 RFM95.

Reads the JSON-per-line stream produced by firmware/davis-track.ino and turns
it into weewx LOOP packets.

The ISS rotates through sensor types -- each transmission carries wind plus
exactly ONE other measurement -- so most LOOP packets are sparse. That is
normal and expected: weewx's accumulators combine them into archive records.

Install:
    cp davisrfm95.py /usr/share/weewx/user/      (or wherever user/ lives)

weewx.conf:
    [Station]
        station_type = DavisRFM95

    [DavisRFM95]
        driver = user.davisrfm95
        port = /dev/ttyACM0
        baudrate = 115200
        # 0.01 in per tip on US models, 0.2 mm (0.007874 in) on metric ones.
        rain_bucket_inches = 0.01
        # Ignore packets weaker than this. See RSSI_FLOOR in the firmware --
        # the value that matters here is the firmware's, this is a backstop.
        min_rssi = -120
"""

import json
import logging
import time

import serial

import weewx.drivers

log = logging.getLogger(__name__)

DRIVER_NAME = "DavisRFM95"
DRIVER_VERSION = "0.1"


def loader(config_dict, _engine):
    return DavisRFM95Driver(**config_dict[DRIVER_NAME])


def confeditor_loader():
    return DavisRFM95ConfEditor()


class DavisRFM95Driver(weewx.drivers.AbstractDevice):
    def __init__(self, **stn_dict):
        self.port_name = stn_dict.get("port", "/dev/ttyACM0")
        self.baudrate = int(stn_dict.get("baudrate", 115200))
        self.rain_bucket = float(stn_dict.get("rain_bucket_inches", 0.01))
        self.min_rssi = float(stn_dict.get("min_rssi", -120))
        self.timeout = int(stn_dict.get("timeout", 60))

        # Rain arrives as a free-running tip counter that wraps at 256, so we
        # report the delta between consecutive readings rather than the value.
        self.last_rain_count = None

        self.serial_port = None
        log.info("DavisRFM95: version %s, port %s", DRIVER_VERSION, self.port_name)

    @property
    def hardware_name(self):
        return DRIVER_NAME

    def openPort(self):
        self.serial_port = serial.Serial(
            self.port_name, self.baudrate, timeout=self.timeout
        )

    def closePort(self):
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None

    def _rain_delta(self, count):
        """Tip-counter delta, accounting for the wrap at 256.

        The first reading only establishes a baseline -- we cannot know how
        much rain fell before we started listening, so it yields nothing.
        """
        if self.last_rain_count is None:
            self.last_rain_count = count
            return None
        delta = (count - self.last_rain_count) % 256
        self.last_rain_count = count
        if delta == 0:
            return None
        if delta > 20:
            # >0.2in between two transmissions ~2.75s apart is not weather,
            # it's a missed-packet artifact or a counter reset. Re-baseline
            # instead of publishing a bogus downpour.
            log.warning("DavisRFM95: implausible rain delta %s tips, ignoring", delta)
            return None
        return delta * self.rain_bucket

    def _to_packet(self, d):
        pkt = {
            "dateTime": int(time.time()),
            "usUnits": weewx.US,
        }

        # Wind rides along in every transmission.
        if "windSpeedMph" in d:
            pkt["windSpeed"] = float(d["windSpeedMph"])
        if "windDirDeg" in d:
            # 0 means north OR a failed vane; Davis does not distinguish. Pass
            # it through -- suppressing it would silently hide a dead sensor.
            pkt["windDir"] = float(d["windDirDeg"])

        if "tempF" in d:
            pkt["outTemp"] = float(d["tempF"])
        if "humidity" in d:
            pkt["outHumidity"] = float(d["humidity"])
        if "uv" in d:
            pkt["UV"] = float(d["uv"])
        if "solarWm2" in d:
            pkt["radiation"] = float(d["solarWm2"])

        if "rainCount" in d:
            rain = self._rain_delta(int(d["rainCount"]))
            if rain is not None:
                pkt["rain"] = rain

        if "rssi" in d:
            pkt["signal1"] = float(d["rssi"])
        if "battLow" in d:
            # weewx convention: 0 = OK, 1 = low.
            pkt["txBatteryStatus"] = int(d["battLow"])

        # rainRate_raw, gust_raw and supercap_raw are deliberately NOT mapped.
        # Their scaling was never verified, and publishing unverified numbers
        # to a public weather network is worse than publishing nothing.
        return pkt

    def genLoopPackets(self):
        if self.serial_port is None:
            self.openPort()

        while True:
            try:
                line = self.serial_port.readline()
            except serial.SerialException as e:
                log.error("DavisRFM95: serial error: %s", e)
                self.closePort()
                time.sleep(5)
                self.openPort()
                continue

            if not line:
                continue
            line = line.decode("utf-8", "replace").strip()
            if not line.startswith("{"):
                # '#'-prefixed status lines from the firmware, or partial reads
                continue

            try:
                d = json.loads(line)
            except ValueError:
                log.debug("DavisRFM95: unparseable line: %r", line)
                continue

            if float(d.get("rssi", 0)) < self.min_rssi:
                continue

            yield self._to_packet(d)


class DavisRFM95ConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[DavisRFM95]
    # This section is for the Davis Vantage Vue via Feather RP2040 RFM95.

    # Serial port of the Feather
    port = /dev/ttyACM0
    baudrate = 115200

    # Rain per bucket tip: 0.01 in (US) or 0.007874 in (0.2 mm, metric)
    rain_bucket_inches = 0.01

    driver = user.davisrfm95
"""

    def prompt_for_settings(self):
        port = self._prompt("port", "/dev/ttyACM0")
        bucket = self._prompt("rain_bucket_inches", "0.01")
        return {"port": port, "rain_bucket_inches": bucket}
