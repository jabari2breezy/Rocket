import unittest
from util.units import convert, format_value

class UnitsTests(unittest.TestCase):
    def test_m_to_ft(self):
        self.assertAlmostEqual(convert(100, "m", "ft"), 328.0839895, places=4)
    def test_ms_to_mph(self):
        self.assertAlmostEqual(convert(44.704, "m/s", "mph"), 100.0, places=3)
    def test_kg_to_lb(self):
        self.assertAlmostEqual(convert(1, "kg", "lb"), 2.2046226218, places=6)
    def test_k_to_c(self):
        self.assertAlmostEqual(convert(288.15, "K", "degC"), 15.0, places=4)
    def test_roundtrip(self):
        self.assertAlmostEqual(convert(convert(5, "m", "ft"), "ft", "m"), 5.0, places=9)
    def test_format_value(self):
        self.assertEqual(format_value(3.14159, "m/s"), "3.14 m/s")

if __name__ == "__main__":
    unittest.main()
