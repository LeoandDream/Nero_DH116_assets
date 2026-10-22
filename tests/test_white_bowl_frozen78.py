"""Pure JSON/schedule checks: importing this helper never imports a simulator."""
from collections import Counter
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / 'examples/right_hand/white_bowl_frozen78.py'
SPEC = importlib.util.spec_from_file_location('white_bowl_frozen78', ENTRY)
FROZEN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FROZEN)


class Frozen78ScheduleTests(unittest.TestCase):
    def test_bound_inputs_use_78_piece_detail_and_original_criteria(self):
        candidate, proof = FROZEN.load_frozen_inputs(ROOT)
        self.assertEqual(proof['candidate_parts'], 78)
        self.assertEqual(proof['finite_sample_count'], 921)
        self.assertFalse(proof['dynamic_control_or_grasp_certified'])
        self.assertEqual(proof['input_sha256'][FROZEN.MODEL_PATH.as_posix()],
                         FROZEN.MODEL_SHA256)
        self.assertEqual(proof['input_sha256'][FROZEN.CANDIDATE_PATH.as_posix()],
                         FROZEN.CANDIDATE_SHA256)
        self.assertEqual(len(proof['input_sha256']), 5)
        self.assertEqual(FROZEN.frozen_criteria(candidate), candidate['proposed_trial_criteria'])

    def test_repeat_only_approach_and_preserve_all_numeric_values_and_order(self):
        candidate, _ = FROZEN.load_frozen_inputs(ROOT)
        original = deepcopy(candidate)
        schedule = FROZEN.build_schedule(candidate)
        self.assertEqual(candidate, original)
        self.assertEqual([row['macro'] for row in schedule], list(range(270)))
        expected_sources = [row['macro'] for row in candidate['commands']
                            for _ in range(2 if row['phase'] == 'approach_high' else 1)]
        self.assertEqual([row['source_macro'] for row in schedule], expected_sources)
        seen = Counter()
        for row in schedule:
            source = original['commands'][row['source_macro']]
            self.assertEqual(row['repeat_index'], seen[row['source_macro']])
            self.assertEqual(row['phase'], source['phase'])
            self.assertEqual(row['action_13'], source['action_13'])
            self.assertEqual(row['target_qpos_18'], source['target_qpos_18'])
            seen[row['source_macro']] += 1
        self.assertEqual(Counter(seen.values()), {1: 190, 2: 40})
        schedule[10]['action_13'][0] = 999
        self.assertEqual(schedule[11]['action_13'], original['commands'][10]['action_13'])
        self.assertEqual(candidate, original)

    def test_phase_boundaries_and_early_stop_budget(self):
        candidate, _ = FROZEN.load_frozen_inputs(ROOT)
        summary = FROZEN.schedule_summary(FROZEN.build_schedule(candidate))
        self.assertEqual(summary['phase_counts'], FROZEN.PHASE_COUNTS)
        self.assertEqual(summary['physics_steps_per_command'], 25)
        self.assertEqual(summary['expected_physics_steps'], 6750)
        self.assertEqual(summary['physics_steps_max'], 6750)
        self.assertEqual(summary['duration_if_completed_s'], 13.5)
        self.assertEqual(summary['phase_boundaries']['approach_high'],
                         {'macro_first': 10, 'macro_last': 89,
                          'physical_step_first': 251, 'physical_step_last': 2250})
        self.assertEqual(summary['phase_boundaries']['hold'],
                         {'macro_first': 240, 'macro_last': 269,
                          'physical_step_first': 6001, 'physical_step_last': 6750})

    def test_reject_old_model_preflight_and_changed_path_or_unknown_result(self):
        reports = [json.loads((ROOT / FROZEN.PREFLIGHT_PATH / name).read_text()) for name in
                   ('internal-detail.json', 'internal-summary.json', 'verification.json')]
        for report_index, field, value in [(0, 'candidate_parts', 17),
                                            (0, 'candidate_sha256', 'old17'),
                                            (0, 'path_sha256', 'changed'),
                                            (1, 'first_unknown', {'sample': 0}),
                                            (1, 'exact_sample_count', 920),
                                            (2, 'frozen_robot_sha256', 'old17')]:
            with self.subTest(field=field):
                changed = deepcopy(reports)
                changed[report_index][field] = value
                with self.assertRaises(ValueError):
                    FROZEN.validate_preflight(*changed)

    def test_changed_original_criteria_and_source_phase_order_are_rejected(self):
        candidate, _ = FROZEN.load_frozen_inputs(ROOT)
        for field, value in [('arm_tracking_error_rad_max', .2),
                             ('continuous_hold_s_min', .5), ('warnings_allowed', 1)]:
            with self.subTest(field=field):
                changed = deepcopy(candidate)
                changed['proposed_trial_criteria'][field] = value
                with self.assertRaises(ValueError):
                    FROZEN.build_schedule(changed)
        changed = deepcopy(candidate)
        changed['commands'][10], changed['commands'][11] = (
            changed['commands'][11], changed['commands'][10])
        with self.assertRaises(ValueError):
            FROZEN.build_schedule(changed)


if __name__ == '__main__':
    unittest.main()
