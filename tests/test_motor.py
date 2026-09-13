import unittest
from physics.motor import (load_motor, isp_effective, mass_flow, MOTOR_CLASSES,
                           impulse_class, total_impulse)
from physics.atmosphere import isa

class MotorTests(unittest.TestCase):
    def test_rp1_climbs_like_real(self):
        self.assertAlmostEqual(isp_effective({"motor_name": "RP1-LOX"}, 0.0), 282, places=0)
        rho = isa(20000.0)["density"]
        expect = 311 - (311 - 282) * rho / 1.225
        self.assertAlmostEqual(isp_effective({"motor_name": "RP1-LOX"}, 20000.0), expect, places=1)
        self.assertAlmostEqual(isp_effective({"motor_name": "RP1-LOX"}, 100000.0), 311, places=0)
    def test_classes_double(self):
        lo_a = MOTOR_CLASSES["A"][0]; lo_c = MOTOR_CLASSES["C"][0]
        self.assertAlmostEqual(lo_c / lo_a, 4.0, places=6)
    def test_solid_isp_constant(self):
        self.assertEqual(isp_effective({"isp": 166, "motor_type": "solid"}, 10000), 166)
    def test_mass_flow_identity(self):
        self.assertAlmostEqual(mass_flow(10.0, 166.0), 10.0 / (166.0 * 9.80665), places=9)
    def test_load_motor_and_classes(self):
        self.assertGreater(load_motor("J250")["total_impulse"], 0)
        self.assertEqual(impulse_class(total_impulse([[0, 0], [1, 10]])), "C")

if __name__ == "__main__":
    unittest.main()