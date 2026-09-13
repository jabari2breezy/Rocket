"""Time-indexed motor curves and propellant consumption."""
from bisect import bisect_right
from physics.atmosphere import G0

def thrust_at(curve, time):
    if time < 0 or time > curve[-1][0]: return 0.0
    i = bisect_right([p[0] for p in curve], time)
    if i == 0: return curve[0][1]
    if i == len(curve): return curve[-1][1]
    t0, f0 = curve[i - 1]; t1, f1 = curve[i]
    return f0 + (f1 - f0) * (time - t0) / (t1 - t0)

def total_impulse(curve):
    return sum((curve[i][1] + curve[i + 1][1]) * (curve[i + 1][0] - curve[i][0]) / 2 for i in range(len(curve) - 1))

def mass_flow(thrust, isp): return thrust / (isp * G0)

from pathlib import Path
import json

def load_motor(name):
    db = json.loads((Path(__file__).resolve().parent.parent / "data" / "motors.json").read_text())
    for family in ("solid", "liquid", "hybrid"):
        if name in db[family]:
            return {"family": family, "name": name, **db[family][name]}
    raise KeyError("unknown motor: %s" % name)

def isp_effective(design, altitude):
    """Return Isp at altitude. Liquids: vac interpolated by density ratio; solids/hybrids fixed."""
    m = {}
    motor_type = design.get("motor_type")
    if motor_type is None or "isp" not in design:
        if "motor_name" in design:
            try:
                m = load_motor(design["motor_name"])
            except KeyError:
                m = {}
        motor_type = m.get("family", design.get("motor_type", "solid"))
    if motor_type in ("solid", "hybrid"):
        return design.get("isp", m.get("isp", 0.0))
    vac = design.get("isp_vac", m.get("isp_vac", 0.0))
    sl = design.get("isp", m.get("isp_sl", m.get("isp", vac)))
    rho0 = 1.225
    from physics.atmosphere import isa
    rho = isa(max(0.0, altitude))["density"]
    return vac - (vac - sl) * min(1.0, rho / rho0)

CLASS_BASE = 1.25
MOTOR_CLASSES = {}
for _i, _letter in enumerate("ABCDEFGHIJKLMNOP"):
    MOTOR_CLASSES[_letter] = (CLASS_BASE * 2 ** _i, CLASS_BASE * 2 ** (_i + 1) - 1e-9)

def impulse_class(total_impulse):
    for letter, (lo, hi) in MOTOR_CLASSES.items():
        if lo <= total_impulse < hi:
            return letter
    return "O"
