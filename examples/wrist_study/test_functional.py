import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import functional as entry


class FrozenCandidateSelection(unittest.TestCase):
    def test_candidate_is_split_and_frozen_adapter_is_restored(self):
        root = entry.ROOT
        fixtures = entry.REFERENCE/'fixtures'
        metadata = json.loads((fixtures/'robot-metadata.json').read_text())
        roles = json.loads((fixtures/'contact-roles.json').read_text())['roles']
        original_split = entry.adapter_module.split_official_model
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            tree = ET.parse(root/'mjcf/nero_dh116.xml')
            ET.SubElement(tree.find('.//body[@name="robot0_link5"]'), 'geom',
                          name='robot0_candidate_probe', type='sphere', size='.001')
            source = folder/'candidate.xml'
            tree.write(source)
            paths = entry.register_candidate(root, folder, metadata, roles, source)['paths']
            self.assertIsNotNone(ET.parse(paths['arm']).find('.//geom[@name="candidate_probe"]'))
        self.assertIs(entry.adapter_module.split_official_model, original_split)
        self.assertIsNone(ET.parse(root/'mjcf/nero_dh116.xml').find('.//geom[@name="robot0_candidate_probe"]'))

    def test_candidate_split_failure_restores_adapter(self):
        original_split = entry.adapter_module.split_official_model
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(FileNotFoundError):
                entry.register_candidate(entry.ROOT, Path(folder), {}, {}, Path(folder)/'absent.xml')
        self.assertIs(entry.adapter_module.split_official_model, original_split)


if __name__ == '__main__':
    unittest.main()
