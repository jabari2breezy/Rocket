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
