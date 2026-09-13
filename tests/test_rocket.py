import unittest
from design.rocket import build
from design.presets import PRESETS

class RocketTests(unittest.TestCase):
    def test_sparky_rollup(self):
        r = build(PRESETS["sparky"])
        self.assertGreater(r["mass"], 0.05)          # ~0.1 kg range
        self.assertLess(r["mass"], 0.5)
        self.assertGreater(r["cg"], 0.0)
        self.assertLess(r["cg"], 0.62)               # inside airframe
        self.assertGreater(r["inertia"], 0.0)

    def test_mass_drops_as_propellant_burns(self):
        full = build(PRESETS["sparky"], prop_remaining=None)
        empty = build(PRESETS["sparky"], prop_remaining=0.0)
        self.assertGreater(full["mass"], empty["mass"])

    def test_cg_moves_with_prop(self):
        full = build(PRESETS["sparky"], prop_remaining=None)
        empty = build(PRESETS["sparky"], prop_remaining=0.0)
        self.assertNotAlmostEqual(full["cg"], empty["cg"], places=4)

    def test_inertia_makes_sense(self):
        r = build(PRESETS["sparky"])
        self.assertGreater(r["inertia"], 0.001)
        self.assertLess(r["inertia"], 0.02)

    def test_all_presets_build(self):
        for k, d in PRESETS.items():
            r = build(d)
            self.assertGreater(r["mass"], 0.01, k)
            self.assertGreater(r["inertia"], 0.0, k)

if __name__ == "__main__":
    unittest.main()