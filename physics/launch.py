"""Launch-rail phase: vertical constraint until the rail ends."""
from physics.atmosphere import isa, gravity
from physics.motor import thrust_at, mass_flow
from physics.aerodynamics import cd_total, reference_area
from physics.motor import load_motor

OFFRAIL_MIN = 30.5   # m/s — real range-safety order of magnitude

def offrail_ok(v_offrail, dive_speed):
    return v_offrail >= max(OFFRAIL_MIN, 4.0 * dive_speed)

def rail_exit(design, rocket, config):
    """Integrate along the rail (purely vertical). Returns dict."""
    rail = design.get("rail_length", 1.2)
    isp = design["isp"]
    curve = design["curve"]
    env0 = isa(0.0)
    t = 0.0
    y = 0.0
    v = 0.0
    prop = design.get("prop_mass", 0.0)
    dt = 0.005
    dry = design.get("casing_mass", 0.0) + design.get("dry_extra", 0.0)
    while y < rail and t < 10.0:
        env = isa(y)
        p = max(0.0, prop)
        T = thrust_at(curve, t) if p > 0 else 0.0
        m_flow = mass_flow(T, isp) if p > 0 else 0.0
        mass = rocket["dry_mass"] + p
        alpha = 0.0
        mach = v / env["sound_speed"] if env["sound_speed"] > 0 else 0.0
        cd = cd_total(design, 0.0, mach, 1.0 if v <= 0 else v * env["density"] * design["length"] / 1.8e-5)
        drag = 0.5 * env["density"] * v * abs(v) * cd * reference_area(design["diameter"])
        a = (T - drag - mass * gravity(y)) / mass if mass > 0 else 0.0
        v += a * dt
        y += v * dt
        prop = max(0.0, prop - m_flow * dt)
        t += dt
    return {"t_exit": t, "v_offrail": v, "y_exit": y}