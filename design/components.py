"""Component mass/CG contributors. Masses come from geometry x material density."""
from math import pi, sqrt

_MATERIALS = None
def _density(material):
    global _MATERIALS
    if _MATERIALS is None:
        import json
        from pathlib import Path
        _MATERIALS = json.loads((Path(__file__).resolve().parent.parent / "data" / "materials.json").read_text())
    return _MATERIALS.get(material, 500.0)

def body_mass(design):
    L, d = design["length"], design["diameter"]
    th = design.get("body_wall_thickness", 0.0015)
    vol = pi * (((d / 2) ** 2) - ((d / 2 - th) ** 2)) * L
    return max(0.001, _density(design.get("body_material", "cardboard")) * vol)

def nose_mass(design):
    d = design["diameter"]
    ln = d * 2.4                       # fineness ~ 2.4
    area = pi * (d / 2) ** 2 + pi * (d / 2) * ln * 0.75   # base + shell
    th = design.get("nose_wall_thickness", 0.0012)
    vol = area * th
    return max(0.0005, _density(design.get("nose_material", "abs_plastic")) * vol)

def fin_mass(design):
    cr, ct = design["fin_root_chord"], design["fin_tip_chord"]
    c = (cr + ct) / 2.0
    vol = design["fin_thickness"] * c * design["fin_span"]
    return design["fin_count"] * _density(design["fin_material"]) * vol

def recovery_mass(design):
    return design.get("recovery_mass", 0.010)

def fin_station(design):
    return design["fin_position"] + design["fin_root_chord"] / 2.0