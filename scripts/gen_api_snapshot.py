#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Write python/ohs/api_1_0.json: the frozen public API of the ohs package.

    PYTHONPATH=build/release/python scripts/gen_api_snapshot.py

Only run this when cutting a new major version (or when adding API in a minor one: the test accepts
additions, so regenerating then just records them). Removing or changing anything already in the
snapshot is a breaking change.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tests", "python"))
from test_api import describe_api  # noqa: E402

out = os.path.join(HERE, "..", "python", "ohs", "api_1_0.json")
with open(out, "w", encoding="utf-8") as f:
    json.dump(json.loads(json.dumps(describe_api())), f, indent=2, sort_keys=True)
    f.write("\n")
print("wrote", os.path.normpath(out))
