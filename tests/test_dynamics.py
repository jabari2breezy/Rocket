import unittest
from physics.dynamics import simulate
from design.presets import PRESETS

def series(sim):
    return sim["series"]

class DynamicsTests(unittest.TestCase):
    def test_sparky_reaches_ballpark(self):
        s = simulate(PRESETS["sparky"])
        self.assertGreater(s["flight"]["apogee"], 80)
        self.assertLess(s["flight"]["apogee"], 250)
    def test_landing_terminates(self):
        s = simulate(PRESETS["sparky"])
        last = series(s)[-1]
        self.assertLessEqual(last["y"], 2.0)
        self.assertGreaterEqual(s["flight"]["flight_time"], 10)
    def test_dt_convergence(self):
        import physics.dynamics as dyn
        d = PRESETS["sparky"]
        a1 = simulate(d)["flight"]["apogee"]
        old_dt = dyn.BURN_DT
        dyn.BURN_DT = old_dt / 2
        try:
            a2 = simulate(d)["flight"]["apogee"]
        finally:
            dyn.BURN_DT = old_dt
        self.assertLess(abs(a2 - a1) / max(a1, 1e-9), 0.001)
    def test_lawn_dart_is_unstable(self):
        s = simulate(PRESETS["lawn-dart"])
        self.assertTrue(s["flight"]["unstable"])
    def test_mach_max_present(self):
        s = simulate(PRESETS["redline"])
        self.assertGreater(s["flight"]["max_mach"], 0.5)
    def test_no_gravity_direction_bug(self):
        s = simulate(PRESETS["sparky"])
        first = series(s)[0]
        self.assertGreaterEqual(first["y"], 0)
    def test_structural_failure_supported(self):
        d = dict(PRESETS["sparky"]); d["max_q_limit"] = 100
        s = simulate(d)
        self.assertIsNotNone(s["failure"])

if __name__ == "__main__":
    unittest.main()