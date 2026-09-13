import unittest
from app import merge_design, ALLOWED_OVERRIDES


class ApiTests(unittest.TestCase):
    def test_override_whitelist(self):
        d = merge_design("sparky", {"length": 0.8, "fin_count": 4, "hack": "x"})
        self.assertEqual(d["length"], 0.8)
        self.assertEqual(d["fin_count"], 4)
        self.assertNotIn("hack", d)

    def test_preset_base(self):
        d = merge_design("sparky", {})
        self.assertEqual(d["name"], "Sparky")

    def test_unknown_preset_falls_back_to_sparky(self):
        d = merge_design("nope", {})
        self.assertEqual(d["name"], "Sparky")

    def test_allowed_override_keys(self):
        for k in ("length", "diameter", "fin_span", "fin_count", "fin_position",
                  "ballast", "wind", "rail_length", "chute_diameter", "max_q_limit",
                  "construction_quality", "fin_root_chord", "fin_tip_chord",
                  "fin_thickness", "fin_material", "body_material"):
            self.assertIn(k, ALLOWED_OVERRIDES)


if __name__ == "__main__":
    unittest.main()