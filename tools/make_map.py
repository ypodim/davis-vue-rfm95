#!/usr/bin/env python3
"""Turn survey.log into the SLOT_MAP for the schedule-following receiver.

Reads every "[found] slot N -> ch C" line (and the final SLOT MAP dump if
present), emits the C array, and sanity-checks the structure we expect:
51 slots, each used channel occupying exactly 3 of them.
"""
import re
import sys
from collections import Counter

LOG = sys.argv[1] if len(sys.argv) > 1 else "survey.log"
SLOTS = 51

slot_map = [-1] * SLOTS
for line in open(LOG):
    m = re.search(r"\[found\] slot (\d+) -> ch (\d+)", line)
    if m:
        slot_map[int(m.group(1))] = int(m.group(2))
    m = re.search(r"^\s*\S*\s*slot (\d+) -> ch (\d+)", line)
    if m:
        slot_map[int(m.group(1))] = int(m.group(2))

known = [c for c in slot_map if c >= 0]
counts = Counter(known)

print(f"identified: {len(known)}/{SLOTS}")
print(f"distinct channels in use: {len(counts)}")
bad = {c: n for c, n in counts.items() if n != 3}
if bad:
    print(f"WARNING: channels not occupying exactly 3 slots: {bad}")
if len(known) == SLOTS and len(counts) * 3 == SLOTS and not bad:
    print("structure check: PASS (17 channels x 3 slots = 51)")

print()
print(f"static const int8_t SLOT_MAP[{SLOTS}] = {{")
for i in range(0, SLOTS, 8):
    row = ", ".join(f"{v:3d}" for v in slot_map[i:i + 8])
    print(f"    {row},")
print("};")

missing = [i for i, v in enumerate(slot_map) if v < 0]
if missing:
    print(f"\nstill unknown slots: {missing}")
