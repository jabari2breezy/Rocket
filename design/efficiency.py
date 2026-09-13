def score(flight, design):
    tw = flight["liftoff_tw"]; margin = flight["min_margin"]; q = flight["max_q"]
    mass_fraction = design["prop_mass"] / (design["dry_mass"] + design["prop_mass"] + design.get("ballast",0))
    subs = {
      "Mass fraction": min(100, mass_fraction / .28 * 100),
      "Thrust / weight": max(0, 100 - abs(min(tw, 15) - 7.5) * 13),
      "Stability": max(0, 100 - abs(margin - 1.5) * 58),
      "Drag loss": max(0, 100 - flight["drag_loss"] * 150),
      "Structure": max(0, 100 - max(0, q - 18000) / 180),
    }
    overall = sum(v*w for v,w in zip(subs.values(), (.25,.20,.20,.20,.15)))
    grade = "S" if overall >= 92 else "A" if overall >= 82 else "B" if overall >= 70 else "C" if overall >= 55 else "D" if overall >= 40 else "F"
    badges=[]
    if flight["max_mach"] >= 1: badges.append(["💥","Mach Buster"])
    if mass_fraction > .25: badges.append(["🪶","Featherweight"])
    if abs(margin-1.5) < .1: badges.append(["🎯","Dead Center"])
    if tw > 15: badges.append(["🔥","Overkill"])
    if flight["landing_speed"] < 5: badges.append(["🪂","Stuck the Landing"])
    return {"grade":grade,"overall":round(overall),"subscores":{k:round(v) for k,v in subs.items()},"badges":badges}
