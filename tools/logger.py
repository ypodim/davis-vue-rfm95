#!/usr/bin/env python3
"""Append serial output from the Feather to a log file, indefinitely.

Usage: python3 logger.py [outfile] [port]
"""
import serial
import sys
import time

path = sys.argv[1] if len(sys.argv) > 1 else "serial.log"
port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM0"

while True:
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
    except Exception:
        time.sleep(2)
        continue
    buf = b""
    with open(path, "a") as f:
        try:
            while True:
                c = ser.read(256)
                if not c:
                    continue
                buf += c
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    f.write(f"{time.time():.2f} {line.decode(errors='replace')}\n")
                    f.flush()
        except Exception:
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(2)
