#!/usr/bin/env python3
"""Compare two directories of LCM logs message by message (channel and payload bytes).

    Cobra/.venv/bin/python tools/compare_logs.py build/acceptance_before build/acceptance

Proves that a change left the default behaviour untouched: run tools/run_acceptance.py with --workdir on the
previous release and on the new build, then compare. The files themselves never hash the same, because LCM
stores the wall-clock time of writing next to every message; that time is ignored here. Exit code 1 if any
message differs.
"""
import os, sys
from lcm import EventLog


def messages(path):
    return [(e.channel, bytes(e.data)) for e in EventLog(path, 'r')]


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    a_dir, b_dir = sys.argv[1], sys.argv[2]
    names = sorted(n for n in os.listdir(a_dir) if n.endswith('.log'))
    missing = [n for n in names if not os.path.exists(os.path.join(b_dir, n))]
    different, total = list(missing), 0
    for n in names:
        if n in missing:
            print(f'{n}: missing in {b_dir}')
            continue
        a, b = messages(os.path.join(a_dir, n)), messages(os.path.join(b_dir, n))
        total += len(a)
        if a != b:
            first = next((i for i, (x, y) in enumerate(zip(a, b)) if x != y), min(len(a), len(b)))
            print(f'{n}: differs ({len(a)} vs {len(b)} messages, first difference at message {first})')
            different.append(n)
    print(f'{len(names)} logs, {total} messages: ' + ('identical' if not different else f'{len(different)} logs differ'))
    sys.exit(1 if different or not names else 0)


if __name__ == '__main__':
    main()
