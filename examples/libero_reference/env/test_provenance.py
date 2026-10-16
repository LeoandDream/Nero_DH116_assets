"""The provenance gate must catch accidental reuse of historical imports."""
import importlib.util
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location("provenance", Path(__file__).with_name("provenance.py"))


class ProvenanceGateTest(unittest.TestCase):
    def load(self):
        if SPEC is None or SPEC.loader is None or not Path(SPEC.origin).is_file():
            self.fail("provenance gate is not implemented")
        module = importlib.util.module_from_spec(SPEC)
        SPEC.loader.exec_module(module)
        return module

    def test_rejects_historical_module_even_with_correct_version(self):
        module = self.load()
        with self.assertRaisesRegex(ValueError, "outside fresh environment"):
            module.validate_module("robosuite", "1.4.1", "1.4.1", "/tmp/old/robosuite/__init__.py", Path("/new/.reference-env"))

    def test_rejects_wrong_package_version(self):
        module = self.load()
        with self.assertRaisesRegex(ValueError, "version"):
            module.validate_module("mujoco", "3.2.3", "3.9.0", "/new/.reference-env/lib/mujoco/__init__.py", Path("/new/.reference-env"))

    def test_accepts_fresh_exact_package(self):
        module = self.load()
        module.validate_module("mujoco", "3.2.3", "3.2.3", "/new/.reference-env/lib/mujoco/__init__.py", Path("/new/.reference-env"))


if __name__ == "__main__":
    unittest.main()
