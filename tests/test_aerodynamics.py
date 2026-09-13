import unittest
from physics.aerodynamics import (cd_total, wave_factor, fin_cn, barrowman,
                                  NOSE_CD_SUBSONIC, reynolds, cd_friction)

def sprocket():
    return {"nose": "Tangent Ogive", "length": 0.62, "diameter": 0.041,
            "fin_span": 0.055, "fin_root_chord": 0.05, "fin_tip_chord": 0.03,
            "fin_count": 3, "fin_position": 0.45}

class AeroTests(unittest.TestCase):
    def test_nose_ranking(self):
        self.assertLess(NOSE_CD_SUBSONIC["Von Kármán"], NOSE_CD_SUBSONIC["Conical"])
    def test_wave_spike(self):
        sub = wave_factor(0.3); at_wall = wave_factor(1.0); sup = wave_factor(1.4)
        self.assertGreater(at_wall, 3.0 * sub)
        self.assertGreater(sub, 1.0)
    def test_cd_rises_with_alpha(self):
        d = sprocket()
        self.assertGreater(cd_total(d, 8, 0.5, 1e6), cd_total(d, 0, 0.5, 1e6))
    def test_fin_cn_positive(self):
        self.assertGreater(fin_cn(sprocket()), 0.0)
    def test_barrowman_cp_aft_of_nose(self):
        cp, n_nb, n_fin, n_tot = barrowman(sprocket())
        self.assertLess(cp, 0.62)
        self.assertGreater(n_fin, 0.0)
        self.assertGreater(n_tot, n_nb)
    def test_reynolds_and_friction(self):
        re = reynolds(1.225, 50, 0.6)
        self.assertAlmostEqual(cd_friction(re), 0.037 / re ** 0.2, places=10)

if __name__ == "__main__":
    unittest.main()
