import unittest
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

class DataTests(unittest.TestCase):
    def _load(self, name):
        return json.loads((ROOT / "data" / name).read_text())

    def test_materials_densities(self):
        d = self._load("materials.json")
        self.assertGreater(d["balsa"], 50)
        self.assertGreater(d["aluminum"], 1000)

    def test_motors_have_curves_and_classes(self):
        m = self._load("motors.json")
        for name in ("solid", "liquid", "hybrid"):
            self.assertIn(name, m)
        c6 = m["solid"]["C6-3"]
        self.assertEqual(c6["class"], "C")
        self.assertGreater(c6["total_impulse"], 5.0)
        self.assertGreater(c6["curve"][1][1], 0)   # first thrust sample nonzero (t=0 is quiescent)

    def test_presets_reference_real_motors(self):
        from design.presets import PRESETS
        m = self._load("motors.json")
        for preset in PRESETS.values():
            if preset.get("motor_type") == "solid":
                self.assertIn(preset.get("motor_name"), m["solid"])

if __name__ == "__main__":
    unittest.main()
