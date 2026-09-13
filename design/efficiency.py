"""S–F efficiency grading: five weighted sub-scores + badges + flavor text."""

WEIGHTS = {"Mass fraction": .25, "Thrust / weight": .20, "Stability": .20,
           "Drag loss": .20, "Structure": .15}

def _clamp(x, lo=0, hi=100):
    return max(lo, min(hi, x))

def score(flight, design):
    needs = ("liftoff_tw", "min_margin", "max_mach", "landing_speed", "drag_loss",
             "max_q", "max_g", "theoretical_dv", "achieved_dv")
    if not all(k in flight for k in needs):
        return {"grade": "F", "overall": 0,
                "subscores": {}, "badges": [],
                "flavor": ["Insufficient telemetry — no grade computed."],
                "explanation": "The flight never produced usable telemetry."}
    prop = design.get("prop_mass", 0.0)
    dry = design.get("dry_mass", prop) + design.get("ballast", 0.0)
    total = dry + prop
    mass_fraction = prop / total if total > 0 else 0.0
    tw = flight["liftoff_tw"]
    margin = flight["min_margin"]
    drag_loss = flight["drag_loss"]
    q = flight["max_q"]
    g = flight["max_g"]
    q_lim = max(design.get("max_q_limit", 1e12) * design.get("construction_quality", 1.0), 1e-9)
    g_lim = max(design.get("max_g_limit", 1e9) * design.get("construction_quality", 1.0), 1e-9)

    # mass fraction vs. motor-type optimum (liquid stages can push ~0.9; solids ~0.25)
    if design.get("motor_type", "solid") in ("liquid", "hybrid"):
        opt = 0.42
    else:
        opt = 0.28
    s_mass = _clamp(100 * mass_fraction / opt)
    # T/W: ideal band 5–10
    if tw <= 0:
        s_tw = 0.0
    elif tw < 5:
        s_tw = _clamp(100 - (5 - tw) * 22)
    elif tw <= 10:
        s_tw = 100.0
    else:
        s_tw = _clamp(100 - (tw - 10) * 12)
    # stability: 1–2 cal throughout, 1.5 ideal
    if margin >= 1.0 and margin <= 2.0:
        s_stab = _clamp(100 - abs(margin - 1.5) * 10)
    elif margin < 1.0:
        s_stab = _clamp((margin / 1.0) * 55)
    else:
        s_stab = _clamp(100 - (margin - 2.0) * 25)
    # drag loss vs theoretical vacuum Tsiolkovsky delta-v
    s_drag = _clamp(100 - drag_loss * 240)
    # structure: fraction of limits used
    s_struct = _clamp(100 - max(q / q_lim, g / g_lim) * 220 + 20)

    subs = {"Mass fraction": round(s_mass), "Thrust / weight": round(s_tw),
            "Stability": round(s_stab), "Drag loss": round(s_drag),
            "Structure": round(s_struct)}
    overall = sum(subs[k] * w for k, w in WEIGHTS.items())
    grade = ("S" if overall >= 92 else "A" if overall >= 82 else "B" if overall >= 70
             else "C" if overall >= 55 else "D" if overall >= 40 else "F")

    flavor = []
    if s_mass < 55:
        flavor.append("You're carrying a lot of dead weight to orbit-that-isn't.")
    if tw < 3:
        flavor.append("This isn't leaving the pad — thrust-to-weight below safe.")
    elif tw > 15:
        flavor.append("Great, you built a cannon — check your off-rail margins.")
    if margin < 1.0:
        flavor.append("CP and CG are having a fight, and CG is losing.")
    if margin > 2.0:
        flavor.append("Overstable — weathercocking will eat your altitude.")
    if s_drag < 55:
        flavor.append("Your rocket is expensive air conditioning.")
    if s_struct < 55:
        flavor.append("This flies great, once.")

    badges = []
    if flight["max_mach"] >= 1.0:
        badges.append(["💥", "Mach Buster"])
    if mass_fraction > 0.92:
        badges.append(["🪶", "Featherweight"])
    if abs(margin - 1.5) < 0.1:
        badges.append(["🎯", "Dead Center"])
    if tw > 15:
        badges.append(["🔥", "Overkill"])
    if flight["landing_speed"] < 5.0:
        badges.append(["🪂", "Stuck the Landing"])

    explanation = ("Mass-fraction-driven: %.0f%% of your sub-scores. "
                   "T/W %.1f, margin %.2f cal, drag loss %.0f%%."
                   % (overall, tw, margin, drag_loss * 100))
    return {"grade": grade, "overall": round(overall), "subscores": subs,
            "badges": badges, "flavor": flavor, "explanation": explanation}