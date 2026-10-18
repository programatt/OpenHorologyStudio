#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Builds the TGZ package with cpack and checks what is in it (and what must not be).

    test_package.py CPackConfig.cmake
"""
import os
import subprocess
import sys
import tarfile
import tempfile
import unittest

CONFIG = None


class PackageContents(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        res = subprocess.run(["cpack", "--config", CONFIG, "-G", "TGZ", "-B", cls.tmp.name], capture_output=True, text=True)
        if res.returncode != 0:
            raise RuntimeError(res.stdout + res.stderr)
        archives = [f for f in os.listdir(cls.tmp.name) if f.endswith(".tar.gz")]
        assert len(archives) == 1, archives
        with tarfile.open(os.path.join(cls.tmp.name, archives[0])) as tar:
            cls.names = tar.getnames()
            cls.root = archives[0][:-len(".tar.gz")]

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def has(self, suffix):
        return any(n == f"{self.root}/{suffix}" or n.endswith("/" + suffix) for n in self.names)

    def test_program_and_data(self):
        for path in ("bin/ohs", "share/ohs/data/materials/materials.toml", "share/ohs/data/dfm/rules.toml",
                     "share/ohs/data/thirdparty/om10/manifest.toml", "share/ohs/data/thirdparty/om10/rig.toml",
                     "share/ohs/scripts/thirdparty/fetch_om10.py"):
            self.assertTrue(self.has(path), path)

    def test_plugin_sdk_and_example(self):
        self.assertTrue(self.has("include/ohs/plugin.h"))
        self.assertTrue(self.has("share/ohs/examples/plugins/min_hole.c"))

    def test_docs_licence_and_integrations(self):
        for path in ("share/doc/ohs/index.md", "share/doc/ohs/lessons.md", "share/doc/ohs/reference/cli.md", "share/doc/ohs/LICENSE.md",
                     "share/doc/ohs/CHANGELOG.md", "share/ohs/integrations/common/ohs_preset.py"):
            self.assertTrue(self.has(path), path)

    def test_python_package_is_shipped(self):
        self.assertTrue(any(n.endswith("/ohs/__init__.py") for n in self.names))
        self.assertTrue(any(n.endswith("/ohs/api_1_0.json") for n in self.names))

    def test_no_cad_files_of_third_parties(self):
        # The OM10 plans are CC BY-SA and must never ship: no STEP files anywhere in the package.
        bad = [n for n in self.names if n.lower().endswith((".step", ".stp", ".igs", ".iges"))]
        self.assertEqual(bad, [])


if __name__ == "__main__":
    CONFIG = sys.argv.pop(1)
    unittest.main()
