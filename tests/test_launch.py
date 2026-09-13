import unittest
from physics.launch import rail_exit, offrail_ok, OFFRAIL_MIN
from design.presets import PRESETS
from design.rocket import build

class LaunchTests(unittest.TestCase):
    def test_offrail_min_constant(self):
        self.assertEqual(OFFRAIL_MIN, 30.5)
    def test_offrail_ok(self):
        self.assertTrue(offrail_ok(35.0, 8.0))
        self.assertFalse(offrail_ok(20.0, 8.0))
    def test_sparky_leaves_rail(self):
        d = PRESETS["sparky"]
        r = build(d)
        res = rail_exit(d, r, {"wind": d["wind"]})
        self.assertGreater(res["t_exit"], 0.05)
        self.assertGreater(res["v_offrail"], 5.0)

if __name__ == "__main__":
    unittest.main()