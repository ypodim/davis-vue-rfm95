#!/usr/bin/env python3
"""Measure link quality at a given location, for antenna/placement experiments.

Reports the two numbers that actually matter:

  * capture rate  -- what fraction of transmissions we actually decoded.
                     This is the metric to optimise. It is measured against
                     the station's known slot interval, so it is absolute
                     rather than relative.
  * RSSI          -- how much margin there is. Useful for comparing two
                     locations that both currently manage 100%, since the
                     one with more margin will survive weather and doors.

Capture rate is the more trustworthy of the two: RSSI is sampled just after
readData() rather than at an ideal point, so treat single readings as noisy
and compare medians over a decent sample.

Usage:
    sudo python3 rssi_survey.py --label "shelf by window" --seconds 180
    sudo python3 rssi_survey.py --label "garage" --port /dev/ttyACM0

Each run appends one row to survey.csv so locations can be compared later,
and writes the raw packet stream to survey-<label>.log.

Note the station's slot interval depends on its transmitter ID:
    interval = (41 + wireID) / 16 seconds
so pass --interval if the ID is not 3 (2.75s).
"""

import argparse
import json
import os
import statistics
import sys
import time

import serial


def summarise(rssis, first_ts, last_ts, interval):
    """Return (n, capture_pct, min, median, max) for a run."""
    n = len(rssis)
    if n < 2:
        return n, 0.0, None, None, None
    elapsed = last_ts - first_ts
    # n-1 gaps between n packets, each nominally one slot
    expected = elapsed / interval + 1
    capture = 100.0 * n / expected if expected > 0 else 0.0
    return n, capture, min(rssis), statistics.median(rssis), max(rssis)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="unlabelled", help="name for this location")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--interval", type=float, default=2.75,
                    help="station slot interval in seconds ((41+wireID)/16)")
    ap.add_argument("--csv", default="survey.csv")
    args = ap.parse_args()

    safe = "".join(c if c.isalnum() or c in "-_" else "-" for c in args.label)
    raw_path = f"survey-{safe}.log"

    print(f"location : {args.label}")
    print(f"port     : {args.port}   interval {args.interval}s")
    print(f"duration : {args.seconds:.0f}s   raw -> {raw_path}")
    print("waiting for packets (blue LED = still searching)...\n")

    ser = serial.Serial(args.port, 115200, timeout=0.5)
    rssis = []
    first_ts = last_ts = None
    buf = b""
    started = time.time()
    last_report = started

    with open(raw_path, "w") as raw:
        while time.time() - started < args.seconds:
            chunk = ser.read(256)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", "replace").strip()
                    raw.write(f"{time.time():.3f} {text}\n")
                    raw.flush()
                    i = text.find("{")
                    if i < 0:
                        continue
                    try:
                        d = json.loads(text[i:])
                    except ValueError:
                        continue
                    if "rssi" not in d:
                        continue
                    now = time.time()
                    if first_ts is None:
                        first_ts = now
                    last_ts = now
                    rssis.append(float(d["rssi"]))

            now = time.time()
            if now - last_report >= 15 and rssis:
                n, cap, lo, med, hi = summarise(rssis, first_ts, last_ts, args.interval)
                print(f"  {now-started:5.0f}s  n={n:4d}  capture={cap:5.1f}%  "
                      f"rssi {lo:.0f}/{med:.0f}/{hi:.0f} (min/med/max)")
                last_report = now

    ser.close()

    if not rssis:
        print("\nNo packets received at all. Either out of range, or the receiver "
              "never acquired -- give it up to ~140s to find the station.")
        sys.exit(1)

    n, cap, lo, med, hi = summarise(rssis, first_ts, last_ts, args.interval)
    print(f"\n=== {args.label} ===")
    print(f"  packets        {n}")
    print(f"  capture rate   {cap:.1f}%")
    print(f"  rssi min/med/max  {lo:.0f} / {med:.0f} / {hi:.0f} dBm")

    new = not os.path.exists(args.csv)
    with open(args.csv, "a") as f:
        if new:
            f.write("timestamp,label,packets,capture_pct,rssi_min,rssi_median,rssi_max\n")
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')},{args.label},{n},"
                f"{cap:.1f},{lo:.0f},{med:.0f},{hi:.0f}\n")
    print(f"\nappended to {args.csv}")

    if os.path.exists(args.csv):
        print("\n--- all locations so far ---")
        with open(args.csv) as f:
            for row in f:
                print("  " + row.rstrip())


if __name__ == "__main__":
    main()
