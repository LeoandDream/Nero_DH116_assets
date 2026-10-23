"""Check left306 default, original V1 preservation and nonwrist equivalence."""
import copy
import hashlib
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
WRIST = ("robot0_link5", "robot0_gripper_flange")

def active(g):
    return int(g.get("contype", "1")) != 0 or int(g.get("conaffinity", "1")) != 0

def canonical(e):
    return (e.tag, tuple(sorted(e.attrib.items())), tuple(canonical(c) for c in e))

def strip_wrist(root):
    root = copy.deepcopy(root)
    root.set("model", "ignored")
    for name in WRIST:
        body = root.find('.//body[@name="' + name + '"]')
        assert body is not None
        removed = [g for g in body.findall("geom") if active(g)]
        mesh_names = {g.get("mesh") for g in removed}
        for geom in removed: body.remove(geom)
        for item in list(root.find("asset")):
            if item.tag == "mesh" and item.get("name") in mesh_names:
                root.find("asset").remove(item)
    return canonical(root)

class LeftWristFine(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.index = json.loads((ROOT / "docs/assets.json").read_text())
        cls.old = ET.parse(ROOT / "mjcf/nero_dh116.xml").getroot()
        cls.new = ET.parse(ROOT / "mjcf/nero_dh116_left_fine.xml").getroot()

    def test_default_and_original_v1(self):
        asset = self.index["assets"]["nero_dh116_left"]
        self.assertEqual(asset["model"], "mjcf/nero_dh116_left_fine.xml")
        self.assertEqual((asset["wrist_collision_proxies"], asset["hand_collision_proxies"]), (306, 76))
        self.assertEqual(hashlib.sha256((ROOT / "mjcf/nero_dh116.xml").read_bytes()).hexdigest(),
                         "c446124b7d938aa6e00ba5914047001de66c2a2c162e7dea1644437e5a85cb33")
        self.assertEqual(hashlib.sha256((ROOT / asset["model"]).read_bytes()).hexdigest(), asset["sha256"])

    def test_nonwrist_fields_unchanged(self):
        self.assertEqual(strip_wrist(self.old), strip_wrist(self.new))

    def test_306_wrist_resource_hashes(self):
        geoms = [g for name in WRIST for g in self.new.find('.//body[@name="' + name + '"]').findall("geom") if active(g)]
        self.assertEqual(len(geoms), 306)
        meshes = {m.get("name"): m for m in self.new.findall("asset/mesh")}
        seen = set()
        for geom in geoms:
            path = (ROOT / "mjcf" / meshes[geom.get("mesh")].get("file")).resolve()
            relative = str(path.relative_to(ROOT))
            self.assertTrue(relative.startswith("meshes/nero/left_wrist_fine/"))
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                             self.index["resources"][relative]["sha256"])
            seen.add(relative)
        self.assertEqual(len(seen), 306)

if __name__ == "__main__":
    unittest.main()
