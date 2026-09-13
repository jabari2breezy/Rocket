"""2-D RK4 trajectory with pitch dynamics, rail → powered → coast → descent, staging."""
from math import pi, sqrt, sin, cos, atan2, log
from physics.atmosphere import isa, gravity
from physics.motor import thrust_at, mass_flow, total_impulse, isp_effective
from physics.aerodynamics import cd_total, reference_area, reynolds, barrowman
from physics.stability import static_margin
from physics.launch import rail_exit, offrail_ok
from physics.heating import stagnation_temp, structural_failure
from design.rocket import build

BURN_DT = 0.01
COAST_DT = 0.05

def _wind_speed(design, y):
    return design["wind"] + design.get("wind_shear", 0.0) * (y / 1000.0)

def _gust(design, t):
    seed = design.get("wind_seed", 0)
    from math import sin as _sin
    return 0.6 * _sin(2.0 * pi * 0.13 * t + seed) + 0.25 * _sin(5.0 * pi * 0.9 * t + seed * 2)

def simulate(design_or_tuple, config=None):
    d_in = design_or_tuple
    if isinstance(design_or_tuple, tuple):
        d_in, config = design_or_tuple
    config = config or {}
    curves = []
    stages = [d_in] + ([d_in["stage2"]] if d_in.get("stage2") else [])
    active = 0
    d = stages[active]
    rocket = build(d)
    prop = d.get("prop_mass", 0.0)
    t = 0.0
    x, y, vx, vy = 0.0, 0.0, 0.0, 0.0
    theta, omega = 0.0, 0.0
    phase = "rail"
    rail_len = d.get("rail_length", 1.2)
    series = []
    warnings = []
    failure = None
    tof = 0.0

    max_q = max_mach = max_alpha = 0.0
    max_stag = max_g = 0.0
    min_margin = 99.0
    unstable = overstable = False
    max_alt = 0.0
    recent = []
    apogee_t = None
    chute_deploy_t = 0.0
    landing_speed = None
    downrange = None
    stage_events = []

    recovery_status = "landed"
    burn_altitude = None
    off_rail_speed = None

    def state_deriv(state, t_now):
        xx, yy, vxx, vyy, th, om, mm = state
        env = isa(max(0.0, yy))
        p_now = max(0.0, mm)
        T_now = thrust_at(d["curve"], t_now) if (phase in ("rail", "powered")) and p_now > 0 else 0.0
        isp_now = isp_effective(d, yy)
        m_dot = -mass_flow(T_now, isp_now) if T_now > 0 else 0.0
        rb = build(d, p_now)
        mass = rb["mass"]
        w = _wind_speed(d, yy) + _gust(d, t_now)
        vrx, vry = vxx - w, vyy
        V = max(1e-6, sqrt(vrx * vrx + vry * vry))
        mach = V / env["sound_speed"] if env["sound_speed"] > 0 else 0.0
        re = reynolds(env["density"], V, d["length"])
        thrust = T_now * isp_now / max(d["isp"], 1e-9)  # m_dot const; F = m_dot * Isp_alt * g0
        alpha = th - atan2(vrx, vry)
        alpha_deg = alpha * 180.0 / pi
        cd = cd_total(d, alpha_deg, mach, re)
        A = reference_area(d["diameter"])
        q = 0.5 * env["density"] * V * V
        drag = q * cd * A
        fx_d = -drag * vrx / V
        fy_d = -drag * vry / V
        cn_total = rb["cn_total"]
        N = q * A * cn_total * alpha
        ux, uy = vrx / V, vry / V
        if abs(alpha) < 1e-6:
            nx, ny = 0.0, 0.0
        else:
            nx, ny = -uy * (1 if alpha > 0 else -1), ux * (1 if alpha > 0 else -1)
        fx_n = N * nx
        fy_n = N * ny
        g_z = gravity(yy)
        bx, by = sin(th), cos(th)
        fxt = thrust * bx
        fyt = thrust * by
        # rail phase: constrain to 1-D vertical
        if phase == "rail":
            fxt = 0.0
            fyt = thrust - drag - mass * g_z
        ax = fx_d / mass + fx_n / mass + fxt / mass
        ay = fy_d / mass + fy_n / mass + fyt / mass - g_z
        if phase == "rail":
            ax = 0.0
        # pitch dynamics (free flight only): stable rocket needs NEGATIVE pitch stiffness
        if phase in ("powered", "coast", "descent"):
            cp_dist = rb["cp"] - rb["cg"]
            restoring = -N * cp_dist                          # N.m; Cm_alpha < 0 for cp > cg
            fin_damp = 0.5 * env["density"] * V * A * d["length"] * 0.02 * om * d["fin_count"]
            m_pitch = restoring - fin_damp
            alpha_dot = m_pitch / max(rb["inertia"], 1e-9)
        else:
            alpha_dot = 0.0
        # descent phase: use chute drag instead of aero drag
        if phase == "descent":
            cd_ch = d.get("chute_cd", 1.0)
            A_ch = pi * (d.get("chute_diameter", 0.0) / 2) ** 2
            if A_ch > 0:
                deploy_t = d.get("chute_deploy_time", 0.5)
                ramp = min(1.0, max(0.0, (t_now - chute_deploy_t) / max(deploy_t, 1e-6)))
                drag_ch = ramp * 0.5 * env["density"] * V * V * cd_ch * A_ch
                fx_d = -drag_ch * vrx / V
                fy_d = -drag_ch * vry / V
                fx_n = 0.0
                fy_n = 0.0
                fxt = 0.0
                fyt = 0.0
                # chute line + riser damp body pitch: no pendulum keeps spinning freely
                alpha_dot = -4.0 * om
        ax = fx_d / mass + fx_n / mass + fxt / mass
        ay = fy_d / mass + fy_n / mass + fyt / mass - g_z
        g_load = sqrt(ax * ax + (ay + g_z) ** 2) / max(g_z, 1e-6)
        return (vxx, vyy, ax, ay, om, alpha_dot, m_dot)

    # pre-integrate rail to get realistic exit velocity
    rl = rail_exit(d, build(d), {"wind": d["wind"]})
    off_rail_speed = rl["v_offrail"]
    if phase == "rail":
        t = rl["t_exit"]
        y = rl["y_exit"]
        vy = rl["v_offrail"]
        vx = 0.0
        theta, omega = 0.0, 0.0
        prop = max(0.0, prop - mass_flow(thrust_at(d["curve"], 0.005), d["isp"]) * t)
        phase = "powered"
        if not offrail_ok(off_rail_speed, 8.0):
            warnings.append("Off-rail speed %.1f m/s below the 8 m/s model guideline — marginal takeoff." % off_rail_speed)

    def rk4_step(state, t_now, h):
        k1 = state_deriv(state, t_now)
        k2 = state_deriv(tuple(state[i] + k1[i] * h / 2 for i in range(7)), t_now + h / 2)
        k3 = state_deriv(tuple(state[i] + k2[i] * h / 2 for i in range(7)), t_now + h / 2)
        k4 = state_deriv(tuple(state[i] + k3[i] * h for i in range(7)), t_now + h)
        return tuple(state[i] + h * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6 for i in range(7))

    while t < 600.0:
        dt = config.get("burn_dt", BURN_DT) if (phase == "powered" and prop > 0) else config.get("coast_dt", COAST_DT)
        state = (x, y, vx, vy, theta, omega, prop)
        # bisect the final burn step so it ends exactly at burnout (prop~0 or thrust curve end)
        if phase == "powered" and prop > 0.0:
            burn_left = max(0.0, d["curve"][-1][0] - t)
            if burn_left > 0.0 and burn_left < dt * 0.999:
                dt = burn_left
        new_state = rk4_step(state, t, dt)
        if phase == "powered" and new_state[6] <= 0.0 and state[6] > 0.0:
            lo, hi = 0.0, dt
            for _ in range(20):
                mid = 0.5 * (lo + hi)
                mid_state = rk4_step(state, t, mid)
                if mid_state[6] > 0.0:
                    lo = mid
                else:
                    hi = mid
            new_state = rk4_step(state, t, hi)
        x, y, vx, vy, theta, omega, prop = new_state
        y = max(0.0, y)
        env = isa(y)
        rb = build(d, prop)
        cp, cg = rb["cp"], rb["cg"]
        margin = static_margin(cp, cg, d["diameter"])
        min_margin = min(min_margin, margin)
        unstable |= margin < 1.0
        overstable |= margin > 2.0
        V = max(1e-6, sqrt((vx - _wind_speed(d, y) - _gust(d, t)) ** 2 + vy ** 2))
        mach = V / env["sound_speed"] if env["sound_speed"] > 0 else 0.0
        q = 0.5 * env["density"] * V * V
        alpha = (theta - atan2(vx - _wind_speed(d, y) - _gust(d, t), vy)) * 180.0 / pi
        max_q = max(max_q, q)
        max_mach = max(max_mach, mach)
        max_alpha = max(max_alpha, abs(alpha))
        max_alt = max(max_alt, y)
        recent.append((t, y))
        if len(recent) > 3:
            recent.pop(0)
        if len(recent) == 3:
            t0, y0 = recent[0]
            t1, y1 = recent[1]
            t2, y2 = recent[2]
            if y1 >= y0 and y1 >= y2 and t1 != t0 and t2 != t1:
                d1 = t1 - t0
                d2 = t2 - t0
                denom = d2 * (d2 - d1)
                if abs(denom) > 1e-12:
                    a = (y2 - y0 - d2 * (y1 - y0) / d1) / denom
                    if abs(a) > 1e-12:
                        b = (y1 - y0 - a * d1 * d1) / d1
                        tv = -b / (2 * a)
                        if 0 <= tv - t0 <= d2:
                            yv = y0 + b * tv + a * tv * tv
                            max_alt = max(max_alt, yv)
        max_stag = max(max_stag, stagnation_temp(env["temperature"], mach))
        ax_now, ay_now = state_deriv((x, y, vx, vy, theta, omega, prop), t)[2:4]
        max_g = max(max_g, sqrt(ax_now * ax_now + (ay_now + gravity(y)) ** 2) / max(gravity(y), 1e-6))

        stag = stagnation_temp(env["temperature"], mach)
        if structural_failure(d, q, max_g):
            failure = {"type": "structural", "at_t": round(t, 2), "q": round(q), "g": round(max_g, 1),
                       "message": "Airframe broke up at q=%.0f Pa, %.1f g." % (q, max_g)}
            break

        if phase == "powered":
            if prop <= 0 or t >= d["curve"][-1][0]:
                phase = "coast"
                burn_altitude = y
                if len(stages) > 1 and active == 0:
                    active = 1
                    d = stages[1]
                    proc = max(prop, 0.0)
                    r_new = build(d, d.get("prop_mass", 0.0))
                    prop = d.get("prop_mass", 0.0)
                    stage_events.append({"t": round(t, 2), "y": round(y, 1), "v": round(V, 1)})
                    warnings.append("Stage 2 ignition at t=%.1f s, %.0f m — stability recomputed." % (t, y))
        elif phase == "coast":
            if vy <= 0 and y > 2.0:
                phase = "descent"
                if apogee_t is None:
                    apogee_t = t
                chute_deploy_t = t

        if (tof == 0 or int(t * 20) % 2 == 0) and len(series) < 4000:
            series.append({"t": round(t, 2), "x": round(x, 2), "y": round(y, 1),
                           "vx": round(vx, 1), "vy": round(vy, 1), "v": round(V, 1),
                           "mach": round(mach, 3), "q": round(q, 0),
                           "cd": round(cd_total(d, alpha, mach, reynolds(env["density"], V, d["length"])), 3),
                           "alpha_deg": round(alpha, 2), "theta_deg": round(theta * 180.0 / pi, 2),
                           "omega": round(omega, 3), "static_margin": round(margin, 3),
                           "cg": round(rb["cg"], 3), "cp": round(rb["cp"], 3),
                           "stagnation_temp": round(stag, 1), "thrust": round(thrust_at(d["curve"], t), 1),
                           "phase": phase, "mass": round(rb["mass"], 4)})

        t += dt
        tof = t
        if failure is not None:
            break
        if (phase == "descent" and y <= 0.01) or (phase in ("coast", "landed") and y <= 0.01 and t > 30):
            landing_speed = V if phase == "descent" else abs(vy)
            downrange = x
            break

    if failure is None and landing_speed is None:
        landing_speed = V
        downrange = x

    initial = build(d_in)["mass"]
    theoretical = isp_effective(d_in, 0) * 9.80665 * log(initial / max(build(d_in)["dry_mass"], 1e-9))
    achieved = max((row["v"] for row in series), default=0.0)
    impulse = total_impulse(d_in["curve"])
    liftoff_tw = thrust_at(d_in["curve"], 0.05) / (initial * 9.80665)

    if landing_speed is not None:
        if landing_speed < 3.0: recovery_status = "recovered"
        elif landing_speed <= 6.0: recovery_status = "serviceable"
        elif landing_speed <= 12.0: recovery_status = "damaged"
        else: recovery_status = "destroyed"
    if recovery_status in ("damaged", "destroyed"):
        warnings.append("Landing speed %.1f m/s — recovery is now structural repair." % landing_speed)

    flight = {
        "apogee": max_alt, "apogee_time": apogee_t or tof, "max_mach": max_mach, "max_q": max_q,
        "max_stagnation": max_stag, "max_g": max_g, "max_alpha_deg": max_alpha,
        "min_margin": min_margin, "margin_class": static_margin(cp, cg, d["diameter"]),
        "unstable": unstable, "overstable": overstable,
        "landing_speed": round(landing_speed, 2) if landing_speed is not None else None,
        "downrange_drift": round(downrange, 1) if downrange is not None else None,
        "off_rail_speed": round(off_rail_speed, 2), "off_rail_ok": offrail_ok(off_rail_speed, 8.0),
        "liftoff_tw": round(liftoff_tw, 2), "burn_time": d_in["curve"][-1][0],
        "burn_altitude": round(burn_altitude, 1) if burn_altitude is not None else None,
        "drag_loss": max(0.0, 1 - achieved / max(theoretical, 1e-9)),
        "theoretical_dv": round(theoretical, 1), "achieved_dv": round(achieved, 1),
        "impulse": round(impulse, 2), "flight_time": round(tof, 2),
        "recovery_status": recovery_status, "stage_events": stage_events,
    }
    flight = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in flight.items()}

    if failure is None:
        from design.efficiency import score as _score
        score_val = _score(flight, d_in)
    else:
        score_val = None
    return {"series": series, "flight": flight, "warnings": warnings,
            "failure": failure, "score": score_val, "cp": round(rb["cp"], 4), "cg": round(rb["cg"], 4),
            "phase": phase}
