#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Compare two ohs_perf result files: perf_compare.py BASELINE.json CURRENT.json [--threshold 0.10].

Prints every benchmark with its change and exits 1 when any got slower by more than the
threshold (default 10 %). Benchmarks missing from either file are reported, never fatal.
"""
import argparse
import json
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("baseline")
    ap.add_argument("current")
    ap.add_argument("--threshold", type=float, default=0.10)
    args = ap.parse_args()
    with open(args.baseline) as f:
        base = json.load(f)
    with open(args.current) as f:
        cur = json.load(f)
    worst = 0.0
    slow = []
    for name in sorted(set(base) | set(cur)):
        if name not in base or name not in cur:
            print(f"{name:42s} only in {'current' if name in cur else 'baseline'}")
            continue
        change = cur[name] / base[name] - 1.0 if base[name] > 0 else 0.0
        mark = ""
        if change > args.threshold:
            mark = "  SLOWER"
            slow.append(name)
        print(f"{name:42s} {base[name]:9.4f} s -> {cur[name]:9.4f} s  {change * 100:+7.1f} %{mark}")
        worst = max(worst, change)
    if slow:
        print(f"\n{len(slow)} benchmark(s) slower than {args.threshold * 100:.0f} %: {', '.join(slow)}")
        return 1
    print("\nno regressions above the threshold")
    return 0


if __name__ == "__main__":
    sys.exit(main())
