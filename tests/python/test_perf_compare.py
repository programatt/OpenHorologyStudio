#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""scripts/perf_compare.py exits 1 on a slowdown above the threshold and 0 otherwise."""
import json
import os
import subprocess
import sys
import tempfile
import unittest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "scripts", "perf_compare.py")


def compare(base, cur, *extra):
    with tempfile.TemporaryDirectory() as tmp:
        a, b = os.path.join(tmp, "a.json"), os.path.join(tmp, "b.json")
        for path, data in ((a, base), (b, cur)):
            with open(path, "w") as f:
                json.dump(data, f)
        return subprocess.run([sys.executable, SCRIPT, a, b, *extra], capture_output=True, text=True)


class PerfCompare(unittest.TestCase):
    def test_no_regression(self):
        r = compare({"x": 1.0, "y": 2.0}, {"x": 1.05, "y": 1.0})
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertIn("no regressions", r.stdout)

    def test_slowdown_is_flagged(self):
        r = compare({"x": 1.0, "y": 2.0}, {"x": 1.25, "y": 2.0})
        self.assertEqual(r.returncode, 1)
        self.assertIn("SLOWER", r.stdout)
        self.assertIn("x", r.stdout.splitlines()[-1])

    def test_threshold_is_adjustable_and_missing_names_are_not_fatal(self):
        self.assertEqual(compare({"x": 1.0}, {"x": 1.25}, "--threshold", "0.5").returncode, 0)
        r = compare({"x": 1.0, "gone": 3.0}, {"x": 1.0, "new": 1.0})
        self.assertEqual(r.returncode, 0)
        self.assertIn("only in baseline", r.stdout)
        self.assertIn("only in current", r.stdout)


if __name__ == "__main__":
    unittest.main()
