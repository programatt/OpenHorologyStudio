#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Check that the commands shown in the documentation exist.

    check_docs.py --ohs build/release/src/ohs [--docs docs] [--readme README.md]

For every fenced code block line (or inline command) that starts with `ohs `, this finds the
subcommand path (the longest run of words that are real subcommands) and checks that every
`--option` used appears in that command's help text. Exits 1 and lists the problems otherwise.
"""
import argparse
import glob
import os
import re
import shlex
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_cli_reference import subcommands  # noqa: E402

_cache = {}


def help_of(ohs, words):
    key = tuple(words)
    if key not in _cache:
        res = subprocess.run([ohs, *words, "--help"], capture_output=True, text=True)
        _cache[key] = res.stdout if res.returncode == 0 else None
    return _cache[key]


def resolve(ohs, tokens):
    """Split tokens into (subcommand words, remaining tokens)."""
    words = []
    text = help_of(ohs, words)
    while text is not None and tokens:
        subs = subcommands(text)
        if tokens[0] in subs:
            words.append(tokens.pop(0))
            text = help_of(ohs, words)
        else:
            break
    return words, tokens, text


def commands_in(path):
    """Command lines starting with `ohs ` inside fenced blocks, joining backslash continuations."""
    with open(path, encoding="utf-8") as f:
        lines = f.read().splitlines()
    in_block, pending, found = False, "", []
    for n, line in enumerate(lines, 1):
        if line.strip().startswith("```"):
            in_block = not in_block
            pending = ""
            continue
        if not in_block:
            continue
        text = pending + line.strip()
        if text.endswith("\\"):
            pending = text[:-1] + " "
            continue
        pending = ""
        if text.startswith("ohs ") or text.startswith("$ ohs "):
            found.append((n, text.lstrip("$ ")))
    return found


def check(ohs, files):
    problems = []
    total = 0
    for path in files:
        for line_no, cmd in commands_in(path):
            total += 1
            cmd = re.split(r"\s+#\s", cmd)[0]           # trailing comments
            cmd = re.split(r"\s*[|>]\s*", cmd)[0]       # pipes and redirects
            try:
                tokens = shlex.split(cmd)
            except ValueError:
                continue
            tokens = tokens[1:]                          # drop "ohs"
            words, rest, text = resolve(ohs, tokens)
            where = f"{os.path.basename(path)}:{line_no}"
            top = help_of(ohs, [])
            first = rest[0] if rest and not words else None
            if not words and first and not first.startswith("-"):
                problems.append(f"{where}: `ohs {first}` is not a command")
                continue
            for tok in rest:
                if tok.startswith("--"):
                    opt = tok.split("=")[0]
                    # Global flags are listed in the top-level help; others in the command's own help.
                    if (text is None or opt not in text) and opt not in (top or ""):
                        problems.append(f"{where}: `ohs {' '.join(words)}` has no option {opt}")
    return total, problems


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ohs", required=True)
    ap.add_argument("--docs", default="docs")
    ap.add_argument("--readme", default="README.md")
    args = ap.parse_args()
    files = sorted(p for p in glob.glob(os.path.join(args.docs, "*.md"))) + [args.readme]
    total, problems = check(args.ohs, [f for f in files if os.path.exists(f)])
    for p in problems:
        print(p)
    print(f"{total} command lines checked, {len(problems)} problem(s)")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
