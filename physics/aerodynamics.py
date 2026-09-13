"""Mach- and AoA-dependent drag build-up + Barrowman CP/CN rollups. SI."""
from math import exp, sqrt, pi

MU = 1.8e-5

# Nose-shape pressure-drag ranking (subsonic, same fineness ratio).
NOSE_CD_SUBSONIC = {
    "Von Kármán": 0.09, "Parabolic": 0.11, "Tangent Ogive": 0.14,
    "Elliptical": 0.18, "Conical": 0.25,
}

def reference_area(diameter):
    return pi * (diameter / 2.0) ** 2

def reynolds(rho, v, length):
    return max(1.0, rho * abs(v) * length / MU)

def cd_friction(re):
    return 0.037 / re ** 0.2

def wave_factor(mach):
    delta = abs(mach - 1.0) / 0.16
    return 1.0 + 2.5 * exp(-min(700.0, delta * delta))

def _fin_drag_coeff(design):
    return 0.018 * design["fin_count"]

def cd_total(design, alpha_deg, mach, re):
    nose = NOSE_CD_SUBSONIC.get(design["nose"], 0.14)
    base = 0.020
    fins = _fin_drag_coeff(design)
    interference = 0.020 * design["fin_count"]
    skin = cd_friction(re)
    cd0 = nose + skin + base + fins + interference
    cd = cd0 * wave_factor(mach)
    cd *= 1.0 + 0.06 * abs(alpha_deg)          # AoA raises drag
    return cd

def fin_cn(design):
    """Barrowman fin normal-force coefficient (rad^-1)."""
    s = design["fin_span"]              # fin half-span (m)
    d = design["diameter"]
    cr = design["fin_root_chord"]
    ct = design["fin_tip_chord"]
    n = design["fin_count"]
    if d <= 0:
        return 0.0
    r = s / d
    c_mid = (cr + ct) / 2.0
    if (cr + ct) <= 0:
        return 0.0
    kfb = 1.0 if cr >= ct else 0.85
    denom = 1.0 + sqrt(1.0 + (2.0 * c_mid / (cr + ct)) ** 2)
    return kfb * (4.0 * n * r * r) / denom

def barrowman(design):
    """Return (CP station m, body CN, fin CN, total CN). Slender-body approx."""
    L = design["length"]
    cn_nb = 2.0                          # slender body dCN/dalpha ~ 2/rad
    x_nb = 0.48 * L
    cn_fin = fin_cn(design)
    x_fin = design["fin_position"] + design["fin_root_chord"] / 2.0
    total = cn_nb + cn_fin
    cp = (cn_nb * x_nb + cn_fin * x_fin) / total if total else x_nb
    return cp, cn_nb, cn_fin, total
