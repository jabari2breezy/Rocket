"""SI<->Imperial conversions. All conversions are exact-factor; no physics here."""
from math import pi  # noqa: F401  (kept for future unit additions)

_CFG = {
    "m":    (1.0,              1.0),
    "ft":   (0.3048,           1.0),
    "m/s":  (1.0,              1.0),
    "mph":  (0.44704,          1.0),
    "kg":   (1.0,              1.0),
    "lb":   (0.45359237,       1.0),
    "m^2":  (1.0,              1.0),
    "ft^2": (0.09290304,       1.0),
    "Pa":   (1.0,              1.0),
    "psi":  (6894.757293168,   1.0),
    "K":    (1.0,              0.0),   # linear: value = scale*si + offset
    "degC": (1.0,              273.15),
}

def convert(value, from_unit, to_unit):
    scale, offset = _CFG[from_unit]
    si = value * scale + offset
    scale, offset = _CFG[to_unit]
    return (si - offset) / scale

def format_value(value, unit, digits=2):
    return "%s %s" % (round(value, digits), unit)

METRIC = {"length": "m", "speed": "m/s", "mass": "kg", "area": "m^2", "pressure": "Pa"}
IMPERIAL = {"length": "ft", "speed": "mph", "mass": "lb", "area": "ft^2", "pressure": "psi"}
