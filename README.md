# Rocket Lab — Hyperrealistic Rocket Simulator

A physics-first, browser-based model-rocket simulator. Design a rocket, launch it,
and read a full engineering debrief: trajectory, Mach, dynamic pressure, stagnation
temperature, structural limits, stability margins, and a scored S–F report card.

## Quick start

```bash
python3 app.py              # serves http://127.0.0.1:5050
```

Open the flight deck at `http://127.0.0.1:5050/`. Launch one of the four curated
presets or build your own, then read the four live charts and the debrief.

The **Tutorial** (`/tutorial`) runs six guided missions that teach you to read the
flight trace, tune stability, and respect structural limits.

## Tests

```bash
python3 -m unittest discover tests -v
```

Pure-Python stdlib throughout — no third-party dependencies, no build step.

## Architecture

```
app.py                 HTTP server (stdlib http.server)
                       routes, validation, whitelisted overrides, JSON API
ui/static/index.html   flight deck (build bay, charts, debrief)
ui/static/tutorial.html mission school (6 guided missions)
ui/static/app.js       d3-style UI wiring, chart rendering (Chart.js 4.4.7)
ui/static/tutorial.js  mission cards + "check my answer" criterion engine
ui/static/units.js     SI / imperial unit conversion helpers
ui/static/style.css    theme
physics/dynamics.py    RK4 6-DOF 2D flight sim (mass burn, chute, staging)
physics/motor.py       motor model + burn curve loader from data/motors.json
physics/atmosphere.py  standard atmosphere (density, temp, Mach terms)
physics/stability.py   CP/CG centers, static margin in calibers
physics/aero.py        drag & lift coefficients across Mach & angle of attack
design/efficiency.py   S–F scoring (5 weighted sub-scores + badges)
design/presets.py      curated designs (Sparky, Redline, Icarus II, Lawn Dart)
data/motors.json       motor burn curves (solid, liquid, hybrid)
tests/                 unittest suite (API, physics, design, URLs)
```

## The physics

- **Trajectory** — 2D RK4 integration of body-frame forces: thrust (from the burn
  curve), drag, lift, gravity, and wind shear. Mass burns off propellant as a
  function of the instantaneous burn progress, so burnout (and every staging
  event) is resolved exactly instead of being smeared across a step.
- **Aerodynamics** — drag and lift coefficients are functions of Mach number and
  angle of attack, with compressibility effects at transonic speeds. Dynamic
  pressure `q`, stagnation temperature, and Mach are logged every step.
- **Stability** — CP and CG are computed from geometry, materials, motor, and
  ballast; static margin is reported in calibers. A rocket is only stable when
  CP sits behind CG and the margin lands in the ~1–2 caliber band.
- **Structure** — the airframe fails when axial load exceeds the material limit
  (thrust × acceleration vs. construction quality). The debrief reports the q
  and g at breakup.
- **Recovery** — a chute deploys on a ramp so deceleration spikes stay physical;
  the deck reports landing speed, downrange drift, and a
  RECOVERED / SERVICEABLE / DAMAGED / DESTROYED outcome.

## API (v2)

| Route                | Method | Purpose |
|----------------------|--------|---------|
| `/`                  | GET    | Flight deck |
| `/tutorial`          | GET    | Mission school |
| `/api/health`        | GET    | Health check |
| `/api/presets`       | GET    | Preset list (stripped of burn curves) |
| `/api/motors`        | GET    | Motor catalog (`solid`/`liquid`/`hybrid`) |
| `/api/flight/run`    | POST   | `{"preset", "overrides"}` → `{design, result}` |
| `/api/design/validate` | POST | Validate a raw design body |

`overrides` is a whitelist — unknown keys, `None` values, and out-of-range
geometry are rejected with a 400 describing the problem. Burn curves are never
served to the client; they stay server-side.

## Mission guide

The debrief grades each flight S–F with five weighted sub-scores and badges.
Key numbers: **apogee** (altitude), **max Mach**, **min static margin** (calibers),
**max q**, **landing speed**, **downrange drift**.

Six tutorial missions teach the loop:

1. **Build Sparky** — launch the intro rocket and read the charts.
2. **Read the grade & radar** — find which sub-score is costing altitude.
3. **Stabilize with nose weight** — push static margin into the 1–2 caliber band.
4. **Push release pressure** — drive an over-thrust design until the structure breaks.
5. **Stage Icarus II** — watch the upper stage ignite at altitude.
6. **Launch The Lawn Dart** — a deliberately unstable design that tumbles.