#!/usr/bin/env python3
"""Pretend to be the Feather: replay a recorded log through a pseudo-terminal.

Lets the whole MQTT/Home Assistant chain be tested on a host that does not
have the hardware attached yet.

    python3 fake_feather.py logs/json-output-100pct-capture.log
    # prints e.g.  /dev/pts/3  -- point davis_mqtt.py --port-dev at it

Lines are emitted at the station's real cadence (one per 2.75 s) unless
--speed says otherwise, and the log loops forever. Only lines carrying JSON
are replayed; the timestamp prefix from tools/logger.py is stripped.
"""

import argparse
import os
import pty
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--interval", type=float, default=2.75)
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
    ap.add_argument("--once", action="store_true", help="do not loop")
    args = ap.parse_args()

    lines = []
    with open(args.log) as f:
        for line in f:
            i = line.find("{")
            if i >= 0:
                lines.append(line[i:].strip())
    if not lines:
        sys.exit("no JSON lines in log")

    master, slave = pty.openpty()
    print(os.ttyname(slave), flush=True)

    delay = args.interval / args.speed
    try:
        while True:
            for line in lines:
                os.write(master, (line + "\n").encode())
                time.sleep(delay)
            if args.once:
                break
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
