#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Install a user-supplied openmovement OM10 STEP file and import it with `ohs`.

The OM10 plans are licensed CC BY-SA 3.0 by openmovement and are never stored in this
repository. Download the STEP file yourself from https://openmovement.org/ (free account),
then run:

    ohs thirdparty om10 fetch --file ~/Downloads/om10-00001_20220701_va_01_3.stp --accept

`--login` (automatic download) is not implemented: openmovement publishes no stable download
URL, so only the manual path is supported for now.
"""
import argparse
import hashlib
import os
import shutil
import subprocess
import sys
import tomllib
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
MANIFEST = REPO / "data/thirdparty/om10/manifest.toml"
PRESET = REPO / "data/thirdparty/om10/rig.toml"

TERMS = """\
The openmovement OM10 plans are published under CC BY-SA 3.0
(https://openmovement.org/general-conditions/). Renders, meshes and modified geometry derived
from them are adaptations: share them alike and credit openmovement. OHS stores your copy in
your per-user data directory only and never in the source tree.
"""


def data_dir() -> Path:
    if sys.platform == "win32":
        base = Path(os.environ.get("LOCALAPPDATA", Path.home() / "AppData/Local"))
    elif sys.platform == "darwin":
        base = Path.home() / "Library/Application Support"
    else:
        base = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share"))
    return base / "ohs/thirdparty/om10"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def known_hashes() -> dict[str, str]:
    if not MANIFEST.exists():
        return {}
    with MANIFEST.open("rb") as f:
        return {e["name"]: e["sha256"] for e in tomllib.load(f).get("file", []) if e.get("sha256")}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", type=Path, help="OM10 .stp you downloaded manually")
    ap.add_argument("--login", action="store_true", help="(not implemented) download automatically")
    ap.add_argument("--accept", action="store_true", help="accept the CC BY-SA 3.0 terms non-interactively")
    ap.add_argument("--ohs", default="ohs", help="path to the ohs executable")
    ap.add_argument("--no-import", action="store_true", help="only install the file")
    args = ap.parse_args()

    if args.login:
        print("error: --login is not implemented; download the file manually and pass --file", file=sys.stderr)
        return 2
    if not args.file:
        ap.error("--file is required")
    if not args.file.is_file():
        print(f"error: {args.file} does not exist", file=sys.stderr)
        return 2

    print(TERMS)
    if not args.accept and input("Accept these terms? [y/N] ").strip().lower() != "y":
        print("terms not accepted; nothing installed")
        return 1

    digest = sha256(args.file)
    expected = known_hashes().get(args.file.name)
    if expected is None:
        print(f"warning: {args.file.name} is not in the manifest (sha256 {digest}); accepting it as a new revision")
    elif expected != digest:
        print(f"warning: {args.file.name} differs from the manifest hash; this may be a modified or newer file")

    dest_dir = data_dir() / args.file.stem
    dest_dir.mkdir(parents=True, exist_ok=True)
    dest = dest_dir / args.file.name
    shutil.copyfile(args.file, dest)
    (dest_dir / "SHA256").write_text(f"{digest}  {args.file.name}\n")
    print(f"installed {dest}")
    if args.no_import:
        return 0

    project = dest_dir / f"{args.file.stem}.hsproj"
    cmd = [args.ohs, "import", str(dest), "-o", str(project), "--beat-rate", "25200"]
    if PRESET.exists():
        cmd += ["--preset", str(PRESET)]
    if subprocess.run(cmd).returncode != 0:
        return 1
    return subprocess.run([args.ohs, "rig", "check", str(project)]).returncode


if __name__ == "__main__":
    sys.exit(main())
