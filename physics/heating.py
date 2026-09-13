"""Aero heating (stagnation temp) and structural failure detection."""

def stagnation_temp(t_ambient, mach):
    return t_ambient * (1.0 + 0.2 * mach * mach)

def structural_failure(design, q, g):
    q_lim = design.get("max_q_limit", 1e12) * design.get("construction_quality", 1.0)
    g_lim = design.get("max_g_limit", 1e9) * design.get("construction_quality", 1.0)
    return q > q_lim or g > g_lim
