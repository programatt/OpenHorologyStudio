#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Write docs/reference/cli.md from the `--help` text of every ohs command.

    gen_cli_reference.py --ohs build/release/src/ohs [--output docs/reference/cli.md]

The reference is generated, never edited by hand, so it cannot drift from the program.
"""
import argparse
import os
import re
import subprocess
import sys


def help_of(ohs, words):
    out = subprocess.run([ohs, *words, "--help"], capture_output=True, text=True, check=True).stdout
    return out.rstrip() + "\n"


def subcommands(help_text):
    """Names listed in the SUBCOMMANDS section of a CLI11 help text."""
    names = []
    in_section = False
    for line in help_text.splitlines():
        if line.strip().lower() == "subcommands:":
            in_section = True
            continue
        if in_section:
            m = re.match(r"^  ([a-z][a-z0-9-]*)\b", line)
            if m:
                names.append(m.group(1))
            elif line.strip() and not line.startswith(" "):
                break
    return names


def walk(ohs, words, depth, out):
    text = help_of(ohs, words)
    title = "ohs " + " ".join(words) if words else "ohs"
    out.append("#" * min(depth + 1, 6) + f" `{title}`\n")
    out.append("```text\n" + text + "```\n")
    for sub in subcommands(text):
        walk(ohs, words + [sub], depth + 1, out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ohs", required=True)
    ap.add_argument("--output", default=os.path.join(os.path.dirname(__file__), "..", "docs", "reference", "cli.md"))
    args = ap.parse_args()
    out = ["# Command-line reference\n",
           "Generated from `ohs --help` by `scripts/gen_cli_reference.py`. Every command accepts the global `--json` flag; "
           "exit code 0 is success, 1 a failed check, 2 bad input.\n"]
    walk(args.ohs, [], 1, out)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        f.write("\n".join(out))
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
