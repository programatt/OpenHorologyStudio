# SPDX-License-Identifier: Apache-2.0
"""Shared logic of the OHS CAD add-ins: turn the parts of a CAD assembly into the role preset
that `ohs import --preset` reads.

The add-ins export the assembly as STEP AP242 and write `<name>.roles.toml` next to it. Each part
gets its role from a custom property called OHS_ROLE (set it on the part in your CAD tool), or,
when there is none, from its name. A preset rule matches the part's exact name, so renaming a
part in the CAD tool and exporting again keeps the roles that you set.
"""
from __future__ import annotations

import re
from typing import Dict, Iterable, List, Optional

# Roles OHS understands (ohs rig set --role ...).
ROLES = (
    "plate", "bridge", "barrel", "mainspring", "arbor", "wheel", "pinion", "escape_wheel",
    "pallet_fork", "balance", "hairspring", "roller", "jewel", "screw",
)

# Name fragments, checked in this order (the first hit wins), the same hints OHS's own classifier uses.
_NAME_RULES = (
    (r"escape", "escape_wheel"),
    (r"pallet|lever|fork", "pallet_fork"),
    (r"hairspring|balance[_ ]?spring", "hairspring"),
    (r"balance", "balance"),
    (r"roller|impulse", "roller"),
    (r"main[_ ]?spring", "mainspring"),
    (r"barrel", "barrel"),
    (r"jewel|ruby|sapphire", "jewel"),
    (r"screw|bolt", "screw"),
    (r"arbor|staff|axle|shaft", "arbor"),
    (r"pinion", "pinion"),
    (r"bridge|cock", "bridge"),
    (r"plate|mainplate|base", "plate"),
    (r"wheel|gear", "wheel"),
)


def escape_regex(text: str) -> str:
    """Escape a part name for use as a literal regex (the preset matches case-insensitively)."""
    return re.escape(text)


def guess_role(name: str) -> Optional[str]:
    """The role a part's name suggests, or None."""
    lowered = name.lower()
    for pattern, role in _NAME_RULES:
        if re.search(pattern, lowered):
            return role
    return None


def role_of(part: Dict[str, str]) -> Optional[str]:
    """A part is a dict with 'name' and optionally 'role' (the OHS_ROLE custom property)."""
    explicit = (part.get("role") or "").strip().lower().replace("-", "_")
    if explicit:
        if explicit not in ROLES:
            raise ValueError(f"part '{part.get('name')}': OHS_ROLE '{explicit}' is not one of {', '.join(ROLES)}")
        return explicit
    return guess_role(part["name"])


def build_preset(parts: Iterable[Dict[str, str]], beat_rate_vph: Optional[float] = None) -> str:
    """The text of a role preset for `ohs import --preset`. Parts without a role are listed as comments."""
    lines: List[str] = ["# Written by an Open Horology Studio CAD add-in. Edit freely.", "# Use: ohs import assembly.step -o movement.hsproj --preset " "<this file>"]
    if beat_rate_vph:
        lines.append(f"# Beat rate of this calibre: {beat_rate_vph:g} vph (pass --beat-rate {beat_rate_vph:g} to ohs import)")
    lines.append("")
    seen = set()
    for part in parts:
        name = part["name"]
        if name in seen:
            continue
        seen.add(name)
        role = role_of(part)
        if role is None:
            lines.append(f"# {name}: no role (set the OHS_ROLE property, or rename the part)")
            continue
        lines += ["[[rule]]", "match = " + _toml_literal(f"^{escape_regex(name)}$"), f'role = "{role}"', ""]
    return "\n".join(lines) + "\n"


def _toml_literal(text: str) -> str:
    """A TOML literal string (single quotes: no escapes), unless the text holds a single quote."""
    if "'" not in text and "\n" not in text:
        return f"'{text}'"
    escaped = text.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def write_preset(path: str, parts: Iterable[Dict[str, str]], beat_rate_vph: Optional[float] = None) -> str:
    text = build_preset(parts, beat_rate_vph)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    return text
