"""Regression for raw recorder annotations on a fixed weld-neighbor contact."""
import copy
import gzip
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / 'examples/right_hand/internal_contact_analysis.py'


class ContactAnnotationTests(unittest.TestCase):
    def test_weld_neighbor_stays_eligible_and_its_distance_is_counted(self):
        self.assertTrue(ENTRY.exists(), 'offline actual-body annotation audit is missing')
        spec = importlib.util.spec_from_file_location('contact_analysis', ENTRY)
        run = importlib.util.module_from_spec(spec); spec.loader.exec_module(run)
        _, candidate = run.CORE.compile_models(run.CORE.derive_xml(run.CORE.REF.MODEL.read_bytes()))
        target = {'gripper0_finger12_link', 'gripper0_finger43_link'}
        with gzip.open(ROOT / 'examples/right_hand/evidence/hand-motion-contacts.jsonl.gz', 'rt') as stream:
            for line in stream:
                row = json.loads(line)
                selected = [c for c in row['contacts'] if set(c['body_names']) == target]
                if selected:
                    example = copy.deepcopy(row); example['contacts'] = [copy.deepcopy(selected[0])]; break
        example['contacts'][0]['enabled_nonadjacent_robot_self'] = False
        example['contacts'][0]['filter_reasons'] = ['EXPLICIT_EXCLUDE']
        audit = run.audit_contacts(candidate, [example])
        self.assertEqual(audit['annotation_mismatch_contacts'], 1)
        self.assertEqual(audit['per_step'][0]['enabled_nonadjacent_self_overlap_m'], -example['contacts'][0]['distance_m'])
        self.assertTrue(audit['per_step'][0]['annotation_mismatches'][0]['actual_enabled_nonadjacent_robot_self'])
        bad = copy.deepcopy(example); bad['contacts'][0]['body_names'][1] = 'gripper0_finger42_link'
        with self.assertRaises(ValueError):
            run.audit_contacts(candidate, [bad])


if __name__ == '__main__':
    unittest.main()
