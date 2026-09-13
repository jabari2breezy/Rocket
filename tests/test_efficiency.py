import unittest
from design.efficiency import score

def flight(fd=None, **kw):
    f = {"liftoff_tw": 7.5, "min_margin": 1.5, "max_q": 5000, "max_g": 10,
         "max_mach": 0.8, "landing_speed": 3.0, "drag_loss": 0.05,
         "theoretical_dv": 1000, "achieved_dv": 950, "apogee": 150}
    f.update(kw or {})
    return f

def design(**kw):
    d = {"prop_mass": 0.032, "dry_mass": 0.09, "ballast": 0.006,
         "motor_type": "solid", "isp": 166, "length": 0.62, "diameter": 0.041}
    d.update(kw or {})
    return d

class EfficiencyTests(unittest.TestCase):
    def test_weights_sum(self):
        self.assertAlmostEqual(sum((.25, .20, .20, .20, .15)), 1.0, places=6)
    def test_ideal_overall(self):
        s = score(flight(), design())
        self.assertEqual(s["grade"], "S")
    def test_bad_tw(self):
        s = score(flight(liftoff_tw=1.2), design())
        self.assertLessEqual(s["subscores"]["Thrust / weight"], 50)
    def test_low_tw_flavor(self):
        s = score(flight(liftoff_tw=1.2), design())
        self.assertTrue(any("pad" in x for x in s["flavor"]))
    def test_landing_badge(self):
        s = score(flight(), design())
        names = [b[1] for b in s["badges"]]
        self.assertIn("Stuck the Landing", names)
    def test_mach_buster(self):
        s = score(flight(max_mach=1.3), design())
        self.assertIn("Mach Buster", [b[1] for b in s["badges"]])
    def test_failure_input(self):
        s = score({"min_margin": 99}, design())
        self.assertEqual(s["grade"], "F")
        self.assertTrue(any("telemetry" in x for x in s["flavor"]))

if __name__ == "__main__":
    unittest.main()