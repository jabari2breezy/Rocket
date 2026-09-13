import unittest
from design.presets import PRESETS
from design.rocket import build
from physics.stability import static_margin
from physics.launch import rail_exit, offrail_ok, OFFRAIL_MIN

class PresetTests(unittest.TestCase):
    def test_all_presets_build(self):
        for k, d in PRESETS.items():
            r = build(d)
            self.assertGreater(r["mass"], 0.01, k)
            self.assertGreater(r["inertia"], 0.0, k)
    def test_sparky_stable(self):
        d = PRESETS["sparky"]; r = build(d)
        m = static_margin(r["cp"], r["cg"], d["diameter"])
        self.assertGreater(m, 1.0)
        self.assertLess(m, 2.0)
    def test_lawn_dart_unstable(self):
        d = PRESETS["lawn-dart"]; r = build(d)
        m = static_margin(r["cp"], r["cg"], d["diameter"])
        self.assertLess(m, 1.0)
    def test_icarus_is_two_stage(self):
        self.assertIn("stage2", PRESETS["icarus"])
    def test_all_motors_loadable(self):
        from physics.motor import load_motor
        for d in PRESETS.values():
            if d["motor_type"] in ("solid",):
                m = load_motor(d["motor_name"])
                self.assertGreater(m["total_impulse"], 0)
    def test_model_scale_offrail(self):
        """Every preset must leave the rail at a credible hobby-model speed (>8 m/s)."""
        for name, d in PRESETS.items():
            r = build(d)
            res = rail_exit(d, r, {"wind": d["wind"]})
            self.assertGreater(res["v_offrail"], 8.0, name)
            self.assertLess(res["t_exit"], 10.0, name)

if __name__ == "__main__":
    unittest.main()