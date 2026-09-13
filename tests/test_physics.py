import unittest
from physics.atmosphere import isa
from physics.motor import thrust_at
from design.presets import PRESETS
from physics.dynamics import simulate

class PhysicsTests(unittest.TestCase):
 def test_isa_sea_level(self): self.assertAlmostEqual(isa(0)["density"],1.225,places=2)
 def test_curve_interpolation(self): self.assertAlmostEqual(thrust_at([[0,0],[1,10]],.5),5)
 def test_sparky_flies(self): self.assertGreater(simulate(PRESETS["sparky"])["flight"]["apogee"],100)
 def test_lawn_dart_warns(self): self.assertTrue(simulate(PRESETS["lawn-dart"])["flight"]["unstable"])
if __name__ == "__main__": unittest.main()
