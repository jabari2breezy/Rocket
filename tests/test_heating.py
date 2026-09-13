import unittest
from physics.heating import stagnation_temp, structural_failure

class HeatingTests(unittest.TestCase):
    def test_stagnation_mach4(self):
        self.assertAlmostEqual(stagnation_temp(216.65, 4.0), 216.65 * (1 + 0.2 * 16), places=6)
    def test_no_failure_under_limits(self):
        d = {"max_q_limit": 100000, "max_g_limit": 40, "construction_quality": 1.0}
        self.assertFalse(structural_failure(d, 50000, 25))
    def test_q_failure(self):
        d = {"max_q_limit": 100000, "max_g_limit": 40, "construction_quality": 1.0}
        self.assertTrue(structural_failure(d, 150000, 25))
    def test_g_failure(self):
        d = {"max_q_limit": 100000, "max_g_limit": 40, "construction_quality": 1.0}
        self.assertTrue(structural_failure(d, 50000, 55))
    def test_quality_reduces_strength(self):
        d = {"max_q_limit": 100000, "max_g_limit": 40, "construction_quality": 0.5}
        self.assertTrue(structural_failure(d, 60000, 25))

if __name__ == "__main__":
    unittest.main()
