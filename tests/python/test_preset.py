# SPDX-License-Identifier: Apache-2.0
"""Tests of the CAD add-in role logic, and a real `ohs import --preset` with the file it writes."""
import json
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "integrations", "common"))
import ohs_preset  # noqa: E402

OHS = os.environ.get("OHS_CLI")


class RoleLogic(unittest.TestCase):
    def test_names_suggest_roles(self):
        cases = {
            "Escape Wheel": "escape_wheel", "third_pinion": "pinion", "Main Plate": "plate", "Barrel": "barrel",
            "balance staff": "balance", "pallet fork": "pallet_fork", "Hairspring": "hairspring", "ruby jewel 2.0": "jewel",
            "train bridge": "bridge", "center wheel": "wheel", "M1.2 screw": "screw", "fourth arbor": "arbor",
            "Oscillating Weight": "rotor", "rotor": "rotor", "rotor screw": "screw", "rotor bridge": "bridge",
        }
        for name, role in cases.items():
            self.assertEqual(ohs_preset.guess_role(name), role, name)
        self.assertIsNone(ohs_preset.guess_role("Body1"))

    def test_explicit_role_wins_and_is_validated(self):
        self.assertEqual(ohs_preset.role_of({"name": "Body1", "role": "Escape-Wheel"}), "escape_wheel")
        self.assertEqual(ohs_preset.role_of({"name": "third wheel", "role": "pinion"}), "pinion")
        with self.assertRaises(ValueError):
            ohs_preset.role_of({"name": "x", "role": "gizmo"})

    def test_preset_text(self):
        text = ohs_preset.build_preset(
            [{"name": "third wheel"}, {"name": "Body1"}, {"name": "third wheel"}, {"name": "pinion (10T)", "role": "pinion"}],
            beat_rate_vph=28800,
        )
        self.assertIn("match = '^third\\ wheel$'", text)
        self.assertIn('role = "wheel"', text)
        self.assertEqual(text.count("[[rule]]"), 2)           # duplicates collapse, unnamed parts become comments
        self.assertIn("# Body1: no role", text)
        self.assertIn("28800 vph", text)
        self.assertIn("pinion\\ \\(10T\\)", text)

    def test_names_with_quotes_stay_valid_toml(self):
        text = ohs_preset.build_preset([{"name": "owner's wheel"}])
        self.assertIn('match = "^owner\'s\\\\ wheel$"', text)


@unittest.skipUnless(OHS and os.path.exists(OHS), "set OHS_CLI to the ohs executable")
class WithTheRealCli(unittest.TestCase):
    def run_ohs(self, *args):
        return subprocess.run([OHS, *args], capture_output=True, text=True)

    def test_import_applies_the_preset(self):
        with tempfile.TemporaryDirectory() as tmp:
            step = os.path.join(tmp, "w.step")
            gen = self.run_ohs("gear", "--teeth", "20", "--module", "0.3", "--bore", "0.8", "--name", "Body1", "--step-out", step)
            self.assertEqual(gen.returncode, 0, gen.stderr)
            preset = os.path.join(tmp, "w.roles.toml")
            ohs_preset.write_preset(preset, [{"name": "Body1", "role": "pinion"}])
            proj = os.path.join(tmp, "w.hsproj")
            imp = self.run_ohs("import", step, "-o", proj, "--preset", preset, "--beat-rate", "25200")
            self.assertEqual(imp.returncode, 0, imp.stderr)
            chk = self.run_ohs("--json", "rig", "check", proj)
            data = json.loads(chk.stdout)
            roles = {c["name"]: c["role"] for c in data["components"]}
            self.assertEqual(roles["Body1"], "pinion")
            # Without the preset the name 'Body1' says nothing and the geometry decides.
            plain = os.path.join(tmp, "plain.hsproj")
            self.assertEqual(self.run_ohs("import", step, "-o", plain).returncode, 0)
            plain_roles = {c["name"]: c["role"] for c in json.loads(self.run_ohs("--json", "rig", "check", plain).stdout)["components"]}
            self.assertEqual(plain_roles["Body1"], "wheel")

    def test_a_bad_preset_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            step = os.path.join(tmp, "w.step")
            self.assertEqual(self.run_ohs("gear", "--teeth", "20", "--module", "0.3", "--step-out", step).returncode, 0)
            bad = os.path.join(tmp, "bad.toml")
            with open(bad, "w") as f:
                f.write("[[rule]]\nmatch = 'x'\nrole = 'gizmo'\n")
            self.assertEqual(self.run_ohs("import", step, "-o", os.path.join(tmp, "x.hsproj"), "--preset", bad).returncode, 2)


if __name__ == "__main__":
    unittest.main()
