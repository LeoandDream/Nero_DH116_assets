"""Frozen command arithmetic has exact phase boundaries and golden actions."""
import gzip
import importlib.util
import json
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent


def load_actions():
    spec = importlib.util.spec_from_file_location('reference_actions', HERE / 'actions.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ActionTests(unittest.TestCase):
    def test_frozen_macro_boundaries_and_endpoints(self):
        initial = {'arm_absolute_qpos_rad': [0.] * 7,
                   'hand_active_source_range_fractions': [.1] * 6}
        plan = {'nodes_contact_to_above': [{'arm_absolute_qpos_rad': [float(i)] * 7} for i in range(11)],
                'open_source_fractions': [.2] * 6, 'closed_source_fractions': [.8] * 6}
        rows = load_actions().generate_actions(initial, plan)
        self.assertEqual(len(rows), 230)
        self.assertEqual([rows[i]['phase'] for i in (0, 9, 10, 49, 50, 119, 120, 159, 160, 199, 200, 229)],
                         ['settle', 'settle', 'approach_high', 'approach_high', 'descend', 'descend',
                          'close', 'close', 'lift', 'lift', 'hold', 'hold'])
        self.assertEqual(rows[49]['action'][:7], [10.] * 7)
        self.assertEqual(rows[119]['action'][:7], [0.] * 7)
        self.assertEqual(rows[120]['action'][7:], [-.6] * 6)
        self.assertEqual(rows[150]['action'][7:], [2*.8-1] * 6)
        self.assertEqual(rows[229]['action'][:7], [10.] * 7)

    def test_all_frozen_commands_match_all_golden_physics_rows(self):
        fixtures = HERE / 'fixtures'
        actions = load_actions().generate_actions(
            json.loads((fixtures / 'initial-joint-config.json').read_text()),
            json.loads((fixtures / 'prepared-white-geometry-plan.json').read_text()))
        golden = fixtures / 'golden' / 'bounded_grasp.jsonl.gz'
        with gzip.open(golden, 'rt') as stream:
            for i, line in enumerate(stream):
                row = json.loads(line)
                self.assertEqual(actions[i // 25]['action'], row['robot_action_13'], i)
                self.assertEqual(actions[i // 25]['phase'], row['phase'], i)
        self.assertEqual(i + 1, 5750)


if __name__ == '__main__':
    unittest.main()
