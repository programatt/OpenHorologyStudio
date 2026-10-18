# SPDX-License-Identifier: Apache-2.0
"""Tests of the Python API, run against the extension module built next to this package."""
import inspect
import json
import os
import tempfile
import unittest

import ohs

HERE = os.path.dirname(os.path.abspath(__file__))
SNAPSHOT = os.path.join(HERE, "..", "..", "python", "ohs", "api_1_0.json")


def describe_api():
    """Public callables with their parameter names, kinds and defaults, as plain data."""
    api = {}
    for name in ohs.__all__:
        obj = getattr(ohs, name)
        if inspect.isclass(obj):
            methods = {}
            for mname, member in inspect.getmembers(obj, predicate=lambda m: inspect.isfunction(m) or isinstance(m, (classmethod, staticmethod))):
                if mname.startswith("_") and mname != "__init__":
                    continue
                methods[mname] = _signature(getattr(obj, mname))
            api[name] = {"kind": "class", "methods": methods}
        elif callable(obj):
            api[name] = {"kind": "function", "signature": _signature(obj)}
        else:
            api[name] = {"kind": "constant", "value": obj}
    return api


def _signature(fn):
    params = []
    for p in inspect.signature(fn).parameters.values():
        if p.name in ("self", "cls"):
            continue
        params.append({"name": p.name, "kind": p.kind.name, "default": None if p.default is inspect.Parameter.empty else repr(p.default),
                       "required": p.default is inspect.Parameter.empty})
    return params


class Freeze(unittest.TestCase):
    def test_public_api_matches_the_frozen_snapshot(self):
        with open(SNAPSHOT, encoding="utf-8") as f:
            frozen = json.load(f)
        current = json.loads(json.dumps(describe_api()))
        # The API is frozen: anything in the snapshot must still be there with the same parameters.
        # New names and new keyword arguments with defaults are allowed in minor versions.
        for name, spec in frozen.items():
            self.assertIn(name, current, f"{name} was removed from the 1.x API")
            self.assertEqual(spec["kind"], current[name]["kind"], name)
            if spec["kind"] == "constant":
                self.assertEqual(spec["value"], current[name]["value"], name)
            elif spec["kind"] == "function":
                self._same_or_extended(name, spec["signature"], current[name]["signature"])
            else:
                for mname, sig in spec["methods"].items():
                    self.assertIn(mname, current[name]["methods"], f"{name}.{mname} was removed")
                    self._same_or_extended(f"{name}.{mname}", sig, current[name]["methods"][mname])

    def _same_or_extended(self, label, old, new):
        self.assertGreaterEqual(len(new), len(old), label)
        for a, b in zip(old, new):
            self.assertEqual(a, b, f"{label}: parameter {a['name']} changed")
        for extra in new[len(old):]:
            self.assertFalse(extra["required"], f"{label}: new parameter {extra['name']} must have a default")

    def test_version(self):
        self.assertEqual(ohs.API_VERSION, "1.0")
        self.assertRegex(ohs.__version__, r"^\d+\.\d+\.\d+")


class Basics(unittest.TestCase):
    def test_materials_and_thermal(self):
        mats = ohs.materials()
        names = [m["name"] for m in mats["items"]]
        self.assertIn("brass_leaded", names)
        t = ohs.thermal("steel_hardened", "steel_hardened")
        self.assertLess(t["rate_s_per_day_per_k"], -8.0)

    def test_errors_raise_and_failed_checks_do_not(self):
        with self.assertRaises(ohs.OhsError) as ctx:
            ohs.thermal("unobtainium")
        self.assertEqual(ctx.exception.exit_code, 2)
        self.assertIn("unknown material", str(ctx.exception))
        with self.assertRaises(ohs.OhsError):
            ohs.run("no-such-command")
        r = ohs.run("verify", "levels", "/nonexistent.hsproj", check=False)
        self.assertEqual(r.exit_code, 2)

    def test_gear_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            r = ohs.gear(24, 0.3, bore=1.0, thickness=0.4, name="w24", step=os.path.join(tmp, "w.step"), dxf=os.path.join(tmp, "w.dxf"),
                         pdf=os.path.join(tmp, "w.pdf"))
            self.assertTrue(r.ok)
            self.assertAlmostEqual(r["tip_diameter"], 7.8, places=6)
            for ext in ("step", "dxf", "pdf"):
                self.assertTrue(os.path.getsize(os.path.join(tmp, "w." + ext)) > 1000)
            with open(os.path.join(tmp, "w.pdf"), "rb") as f:
                self.assertTrue(f.read(8).startswith(b"%PDF-1.4"))
            with self.assertRaises(ohs.OhsError):
                ohs.gear(3, 0.3, dxf=os.path.join(tmp, "x.dxf"))

    def test_lessons(self):
        listing = ohs.lessons()
        ids = [l["id"] for l in listing["items"]]
        self.assertEqual(ids[:2], ["train", "timegrapher"])
        shown = ohs.lesson("train")
        self.assertEqual(shown["id"], "train")
        self.assertGreaterEqual(len(shown["steps"]), 3)
        sol = ohs.check_lesson("train", solution=True)
        passed = ohs.check_lesson("train", answer=float(sol["solution"]))
        self.assertTrue(passed["passed"])
        failed = ohs.check_lesson("train", answer=1.0)
        self.assertFalse(failed["passed"])
        self.assertEqual(failed.exit_code, 1)
        self.assertFalse(failed.ok)

    def test_calibre_template(self):
        text = ohs.calibre_template(28800)
        self.assertIn("[calibre]", text)
        self.assertIn("beat_rate_vph = 28800", text)

    def test_verify_physics_fast_set(self):
        r = ohs.verify_physics()
        self.assertTrue(r["ok"])
        self.assertGreaterEqual(len(r["checks"]), 4)


class WithProject(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        step = os.path.join(cls.tmp.name, "m.step")
        ohs.run("dev", "gen-movement", "-o", step, json=False)
        cls.project = ohs.Project.import_step(step, os.path.join(cls.tmp.name, "m.hsproj"), beat_rate=25200)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_kinematics_and_explain(self):
        k = self.project.kinematics()
        self.assertEqual(k["escape_teeth"], 15)
        rpm = sorted(m["rpm"] for m in k["mobiles"])
        self.assertAlmostEqual(rpm[-1], 14.0, places=6)
        walk = self.project.explain(train=True)
        self.assertEqual(len(walk["train"]["steps"]), 4)
        self.assertEqual(self.project.rig_check()["errors"], 0)

    def test_timekeeping_family(self):
        t = self.project.torque()
        self.assertAlmostEqual(t["speed_ratio"], 6720)
        tk = self.project.timekeeping()
        self.assertGreater(tk["power_reserve_h"], 40)
        sim = self.project.simulate(duration="30s")
        self.assertGreater(sim["result"]["amplitude_deg"], 240)
        tg = self.project.timegrapher(duration="20s", jitter_us=20, seed=4)
        self.assertGreater(tg["amplitude_deg"], 240)
        self.assertEqual(tg, self.project.timegrapher(duration="20s", jitter_us=20, seed=4))   # seeded

    def test_whatif_with_changes_and_validation(self):
        r = self.project.whatif(changes=[{"parameter": "hairspring_length_mm", "scale": 1.002}])
        rate = [m for m in r["metrics"] if m["name"].startswith("rate, dial up")][0]
        self.assertAlmostEqual(rate["change"], -86.4, delta=3.0)   # 0.2 % longer: sqrt rule, 0.1 % of 86400 s
        with self.assertRaises(ohs.OhsError):
            self.project.whatif(changes=[{"parameter": "hairspring_length_mm"}])
        with self.assertRaises(ohs.OhsError):
            self.project.whatif()
        with self.assertRaises(ohs.OhsError):
            self.project.whatif(changes=[{"parameter": "warp_factor", "set": 9}])

    def test_tolerance_exit_code_is_a_result_not_an_exception(self):
        r = self.project.tolerance(samples=2000)
        self.assertEqual(r.exit_code, 1)
        self.assertFalse(r.ok)
        self.assertGreater(r["failing_fraction"], 0.05)
        self.assertTrue(self.project.tolerance(samples=2000, fail_above=0.9).ok)

    def test_tooth_bending(self):
        r = self.project.fea_tooth("third_wheel")
        self.assertEqual(r["teeth"], 75)
        self.assertGreater(r["concentration_over_lewis"], 1.0)

    def test_missing_project(self):
        with self.assertRaises(ohs.OhsError):
            ohs.Project("/nonexistent.hsproj")


if __name__ == "__main__":
    unittest.main()
