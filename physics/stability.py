"""Static margin bands and pitch inertia."""

def static_margin(cp, cg, diameter):
    if diameter <= 0:
        return 0.0
    return (cp - cg) / diameter

def margin_class(margin):
    if margin < 1.0:
        return "unstable"
    if margin <= 2.0:
        return "stable"
    return "overstable"

def margin_health(margin):
    if margin < 1.0:
        return "DANGER"
    if margin <= 2.0:
        return "OK"
    return "WARN"

def pitch_inertia_from_parts(parts, cg):
    return sum(m * (x - cg) ** 2 for _, m, x in parts)
