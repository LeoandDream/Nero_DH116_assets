"""The diagnostic intervention changes one saved command channel only."""
import copy
import gzip
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


def module():
    spec = importlib.util.spec_from_file_location('thumb_coordination', ROOT / 'examples/right_hand/thumb_coordination.py')
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


class CoordinationPlanTests(unittest.TestCase):
    def setUp(self):
        with gzip.open(ROOT / 'examples/right_hand/evidence/frozen-commands.json.gz', 'rt') as stream:
            self.original = json.load(stream)['hand_motion'][:1000]

    def test_only_finger12_changes_and_saved_plan_is_not_mutated(self):
        saved = copy.deepcopy(self.original)
        actual = module().coordination_plan(self.original)
        self.assertEqual(saved, self.original)
        self.assertEqual(len(actual), 1000)
        changed = 0
        for old, new in zip(saved, actual):
            expected = copy.deepcopy(old)
            expected['fractions'][1] = old['fractions'][0]
            self.assertEqual(expected, new)
            changed += new['fractions'][1] != old['fractions'][1]
        self.assertEqual(changed, 749)

    def test_rejects_wrong_length_or_another_active_channel(self):
        run = module()
        with self.assertRaises(ValueError):
            run.coordination_plan(self.original[:-1])
        wrong = copy.deepcopy(self.original)
        wrong[100]['fractions'][2] = .1
        with self.assertRaises(ValueError):
            run.coordination_plan(wrong)

    def test_rejects_a_changed_schedule_before_any_physics(self):
        wrong = copy.deepcopy(self.original)
        wrong[100]['steady'] = True
        with self.assertRaises(ValueError):
            module().coordination_plan(wrong)


if __name__ == '__main__':
    unittest.main()
