#!/usr/bin/env python3
"""Download and analyse site-survey sessions from the Feather's flash.

Companion to firmware-survey/davis-survey-logger.ino. Carry the Feather on
battery to a candidate location, let it record, bring it back, then:

    sudo python3 survey_download.py                    # list all sessions
    sudo python3 survey_download.py --label-last "garage"
    sudo python3 survey_download.py --clear            # erase after downloading

Reports per session:
  capture % -- fraction of transmissions actually decoded, measured against
               the station's known slot interval. Optimise this first.
  RSSI      -- the margin. Once capture saturates near 100%, this is what
               distinguishes a merely-working location from a robust one.
"""

import argparse
import json
import statistics
import sys
import time

import serial

SLOT_S = 2.75      # (41 + wireID)/16 for wire ID 3


def read_dump(port, timeout=25):
    """Ask the device for its stored log and return the raw text."""
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"d")
    ser.flush()

    out, started, deadline = [], False, time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", "replace").strip()
        if not line:
            continue
        if line.startswith("#EMPTY"):
            ser.close()
            return None
        if line.startswith("#BEGIN"):
            started = True
            continue
        if line.startswith("#END"):
            ser.close()
            return "\n".join(out)
        if started:
            out.append(line)
    ser.close()
    return "\n".join(out) if out else None


def parse_sessions(text):
    sessions, cur = [], None
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("#SESSION"):
            cur = {"header": line, "samples": []}
            sessions.append(cur)
        elif line.startswith("{") and cur is not None:
            try:
                cur["samples"].append(json.loads(line))
            except ValueError:
                pass
    return sessions


def report(sessions, interval):
    for i, s in enumerate(sessions, 1):
        smp = s["samples"]
        print(f"\n=== session {i} ===   {s['header']}")
        if len(smp) < 2:
            print("  too few packets to summarise "
                  "(station may have been out of range)")
            continue
        rssis = [x["rssi"] for x in smp]
        span = (smp[-1]["ms"] - smp[0]["ms"]) / 1000.0
        expected = span / interval + 1
        capture = 100.0 * len(smp) / expected if expected > 0 else 0.0
        # consecutive channel steps == no missed slot between two packets
        chans = [x["ch"] for x in smp]
        steps = [(b - a) % 51 for a, b in zip(chans, chans[1:])]
        consec = steps.count(1)
        print(f"  packets      {len(smp)} over {span:.0f}s")
        print(f"  capture      {capture:.1f}%   "
              f"(consecutive slots {consec}/{len(steps)})")
        print(f"  rssi         min {min(rssis)}  median {statistics.median(rssis):.0f}  "
              f"max {max(rssis)} dBm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--interval", type=float, default=SLOT_S)
    ap.add_argument("--save", default="survey-sessions.jsonl")
    ap.add_argument("--clear", action="store_true",
                    help="erase the device log after a successful download")
    args = ap.parse_args()

    print(f"reading from {args.port} ...")
    text = read_dump(args.port)
    if not text:
        print("No stored sessions. Either nothing was recorded yet, or the log "
              "was cleared.\nNote the logger only writes to flash once the "
              "3-minute recording completes (purple LED) -- powering off during "
              "the blue or green phase saves nothing.")
        sys.exit(1)

    with open(args.save, "w") as f:
        f.write(text + "\n")
    sessions = parse_sessions(text)
    print(f"downloaded {len(sessions)} session(s) -> {args.save}")
    report(sessions, args.interval)

    if args.clear:
        ser = serial.Serial(args.port, 115200, timeout=1)
        time.sleep(0.3)
        ser.write(b"c")
        ser.flush()
        time.sleep(0.5)
        ser.close()
        print("\ndevice log cleared")


if __name__ == "__main__":
    main()
