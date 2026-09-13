import unittest
from physics.stability import static_margin, margin_class, margin_health

class StabilityTests(unittest.TestCase):
    def test_margin_formula(self):
        self.assertAlmostEqual(static_margin(cp=0.48, cg=0.42, diameter=0.041), 0.06 / 0.041, places=4)
    def test_class_bands(self):
        self.assertEqual(margin_class(0.5), "unstable")
        self.assertEqual(margin_class(1.1), "stable")
        self.assertEqual(margin_class(2.5), "overstable")
    def test_health(self):
        self.assertEqual(margin_health(0.9), "DANGER")
        self.assertEqual(margin_health(1.0), "OK")
        self.assertEqual(margin_health(2.1), "WARN")

if __name__ == "__main__":
    unittest.main()
