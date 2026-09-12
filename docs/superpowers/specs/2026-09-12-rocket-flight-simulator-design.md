# Rocket Flight Simulator — Design Spec

Date: 2026-09-12
Status: Approved (pending spec review)

## 1. Overview

A physics-accurate rocket design & flight simulator. Users assemble a rocket from real
components (nose cone, body tube, fins, motor, recovery system), hit **Launch**, and the
simulator flies it through a layered ISA atmosphere using real aerospace equations
(Barrowman stability, Mach-dependent drag, RK4 integration). Every flight earns an
**Efficiency Rating** (S–F) built from five weighted sub-scores, so a bad grade always
tells you *which subsystem* is holding you back. The app is a teaching tool: presets
range from a sane baseline rocket up to an intentionally unstable "Lawn Dart" so users
can watch failure modes happen safely.

## 2. Goals & Non-Goals

### Goals
- Physics must be real, inspectable, and unconverged-nothing: ISA atmosphere, thrust
  curves, Mach-dependent drag build-up, Barrowman CP/CN, CG recomputed every timestep,
  RK4 integration, altitude-varying gravity.
- A web UI that is fully decoupled from the physics core (pure functions in, state
  time-series out).
- A beginner-to-advanced tutorial embodied as in-app guided missions plus a README.
- Every result self-diagnoses: letter grade + radar chart + per-subsystem flavor text.

### Non-Goals (this iteration)
- Full 3D trajectory (yaw/azimuth) — flights live in a 2D vertical plane.
- Real-time animated launch playback — results are post-flight plots.
- Aerodynamic interference factors beyond engineering-fair approximations; no CFD.
- Structural/FEM, thermal, or pie-in-the-sky-ness (e.g. electric propulsion).
- User accounts, cloud flight sharing, or persistent saved designs across sessions
  (designs export/import as JSON files only).

## 3. Scope Decisions (Confirmed)

| Decision | Choice |
|---|---|
| Delivery | Physics core + pytest regression tests + web UI, pushed to GitHub |
| Tutorial | In-app guided missions built on the presets + README physics walkthrough |
| Flight display | Post-flight results dashboard (instant, deterministic) |
| Units | SI everywhere internally; UI toggle for Imperial (ft / mph / lb) |
| Trajectory | 2D vertical plane (rocket + wind coplanar); side-profile arc |
| Stack approach | Zero-framework: Python stdlib `http.server` + NumPy + chart.js CDN |

## 4. Architecture

### 4.1 Runtime Stack
- **Physics core:** Python 3.9 + NumPy. Hand-rolled RK4 integrator — not a black-box
  ODE solver, so every step is inspectable.
- **Web server:** Python stdlib `http.server` `ThreadingHTTPServer`. Serves static
  `ui/` assets and a JSON API:
  - `GET /` — builder + dashboard UI (HTML/CSS/JS)
  - `GET /tutorial` — tutorial page
  - `POST /api/design/save` — validate a design, echo its component/mass/CG/CP rollup
  - `POST /api/flight/run` — run the full sim, return flight time-series + scoring
  - `POST /api/design/import` / `GET /api/design/export?id=` — JSON import/export
  - `GET /api/motors` — motor catalog for UI dropdowns
- **Frontend:** plain HTML/CSS/JS. chart.js via CDN for plots + radar. Inline SVG for
  the rocket side-profile rendered from design components. No browser-side physics.
- **Data:** `data/motors.json`, `data/materials.json`, `data/presets.json` — presets are
  data files; sharing a design = sharing a text file.

### 4.2 Module Layout

```
physics/
    atmosphere.py     — ISA lookups (0–86 km), ρ(z), a(z), g(z), mach
    motor.py          — thrust-curve interpolation, mass flow, Isp, v_e
    aerodynamics.py   — Cd build-up vs (Mach, α), Barrowman CN/CP
    dynamics.py       — equations of motion, RK4 integrator, flight driver
    stability.py      — CG tracking, static margin, stability state
    recovery.py       — chute deployment, descent phase
design/
    components.py     — NoseCone/BodyTube/Transition/Fin/MotorMount/Recovery/Payload
    rocket.py         — assemble components → mass/CG/CN/CP rollups; staging splits
    presets.py        — Sparky, Redline, Icarus II, The Lawn Dart
    efficiency.py     — S–F grading, radar sub-scores, badges, flavor text
util/
    units.py          — SI↔Imperial conversions at the API/UI edge
ui/
    index.html / tutorial.html / app.js / charts.js / rocket-svg.js / style.css
data/
    motors.json       — real thrust curves (C6 family); solid A–O ranges; liquid pairs
    materials.json    — material densities (masses from volumes)
    presets.json      — the four presets as data
tests/
    test_atmosphere.py
    test_motor.py
    test_aerodynamics.py
    test_dynamics.py
    test_stability.py
    test_design.py
    test_efficiency.py
    test_regression_c6.py
    test_units.py
docs/
    README.md         — physics walkthrough + tutorial + repo guide
```

### 4.3 Data Flow

```
UI form / preset JSON
      │  (design dict, SI)
      ▼
design.rocket.build(design) → Rocket (mass, CG, per-body components, staging plan)
      │
      ▼
physics.flight_sim.simulate(rocket) → FlightResult time-series
      │   (RK4; atmosphere+aero+stability per timestep; recovery; staging)
      ▼
design.efficiency.score(design, flight) → Letter grade, 5 sub-scores, badges, warnings
      │
      ▼
JSON response → UI renders plots / radar / SVG profile / warnings
```

## 5. Physics Spec

### 5.1 State & Integration
3-DOF point-mass + pitch: `(x, y, z, vx, vy, vz, θ, ω, m)`. RK4 at dt=0.01 s during
burn, relaxed to dt=0.05 s during coast (post-burn, pre-apogee). No Euler integration.
Timestep-convergence check: halving dt must move apogee < 0.1%.

### 5.2 Thrust & Motor
- Thrust = time-indexed curve, linearly interpolated. Not a flat average.
- `Isp = I_total / (m_prop × g0)`, `g0 = 9.80665`, `v_e = Isp × g0`, `ṁ(t)=T(t)/v_e`.
- Motor database (`data/motors.json`):
  - **Solid** A–O impulse classes (each class ~2× the last; A ≈ 1.26–2.5 N·s …
    O ≈ 40,961–81,920 N·s), Isp 180–240 s, with sampled real thrust profiles for the
    presets (C6 family used by Sparky). Published values only.
  - **Liquid** custom builder: RP-1/LOX (282 s SL / 311 s vac), LH2/LOX (366/452 s),
    hypergolic N2O4/UDMH (285/336 s) — show the Isp/delta-v effect of propellant swap.
  - **Hybrid** HTPB/N2O preset (200–250 s).

### 5.3 Drag
`Cd_total(Mach, α) = Cd_friction + Cd_nose + Cd_base + Cd_wave + Cd_fins + Cd_interference`
- Skin friction (turbulent): `Cf = 0.037/Re^0.2`, `Re = ρ v L / μ`; friction drag from
  wetted area.
- Nose shapes with subsonic relative-Cd table: Von Kármán/LV-Haack (lowest), Parabolic
  (very low), Tangent ogive (low-moderate), Elliptical (moderate), Conical (highest).
- Transonic wave drag spike 3–4× subsonic Cd across Mach 0.8–1.2, falling after Mach 1.
- Fins + interference terms scaled by fin geometry and body-fin matching.
- Cd is a function of Mach *and* angle of attack, never a bare constant.

### 5.4 Stability (Barrowman)
- `CN_fins = Kfb × [4n(s/d)² / (1 + √(1 + (2l/(Cr+Ct))²))]`
- `CP = Σ(CNᵢ·Xᵢ)/Σ(CNᵢ)`, `Static margin = (CP − CG)/d`
- 1–2 calibers = stable; <1 = red instability flag; >2 = yellow overstable/weathercock.
- CG recomputed **every timestep** (burning propellant moves CG, mainly in flights with
  forward motors). A stable liftoff can become unstable at burnout — surfaced as warning.

### 5.5 Atmosphere
Real ISA, layered lapse rates, 0–86 km (troposphere … mesosphere exactly per the
reference table), `ρ = P/(R_spec·T)`, `R_spec = 287.05`, `a = √(1.4·R_spec·T)`,
`g(z) = 9.80665·(R_e/(R_e+z))²` with `R_e = 6,371 km`. ISA + seasonal/day offsets not
modeled.

### 5.6 Wind & Flight Feel
2D vertical plane. Wind vector (constant component) + optional altitude shear +
gust turbulence feed the relative-velocity term used for drag *and* the AoA driving
weathercocking. Wind strength selectable per launch; gusts modeled as time-varying
sinusoidal + noise terms on the wind vector.

### 5.7 Recovery
Parachute deploys automatically at apogee (no freefall delay): `F = 0.5 ρ v² Cd_chute
A_chute`, `Cd_chute ≈ 0.75–1.5` (selectable). Warn if landing speed > 6–7 m/s
("recovery" becoming "structural repair"). Post-deploy the rocket descends nearly
vertically under drag until ground contact ends the sim.

### 5.8 Staging
At separation: drop dry mass instantly, carry velocity/position forward, recompute
mass/CG/stability **from scratch** for the new vehicle. Second stage = a new stability
problem riding the first stage's trajectory. Up to 2 stages (covers "Icarus II").

## 6. Design System

Components: nose cone (6 shapes), body tube(s), transitions/boattails, fins
(trapezoidal/elliptical/freeform: span, chord, sweep, thickness, material), motor mount,
recovery system, payload/ballast (enables nose-weighting to fix stability — the cheapest
real fix, and a teaching moment).

Design rollups (`design/rocket.py`): total mass; CG per component; CN/CP per component
and total; per-timestep CG during burn; staging split.

### Presets
| Preset | Teaches |
|---|---|
| Sparky | Sane baseline; A–C motor, balsa fins |
| Redline | High-power, H–L motor, dual-deploy, breaks Mach 1; transonic drag, max-Q |
| Icarus II | Two-stage; staging logic + stability recalculation |
| The Lawn Dart | Intentionally unstable (CP ahead of CG); guided failure demo |

## 7. Efficiency Rating (S, A, B, C, D, F)

Five sub-scores (0–1 → mapped to letter), weighted:
| Sub-score | Weighs | Weight |
|---|---|---|
| Mass Fraction | ζ=(m0−mf)/m0 vs. motor-type optimum | 25% |
| Thrust-to-Weight | Liftoff T/W in 5:1–10:1 band | 20% |
| Stability | Static margin in 1–2 calibers through burn | 20% |
| Drag Loss | achieved vs. vacuum Tsiolkovsky Δv | 20% |
| Structural Margin | max-Q & peak g vs. assumed airframe strength | 15% |

`Overall = Σ(weight × sub-score)`; radar chart so lopsided designs look lopsided.
Failing sub-scores emit subsystem-specific flavor text (e.g. "Your rocket is expensive
air conditioning."). Grade boundaries: ≥0.9 S, ≥0.75 A, ≥0.6 B, ≥0.45 C, ≥0.3 D, else F.

Badges: 🏆 Mach Buster (M>1), 🪶 Featherweight (ζ>0.92), 🎯 Dead Center (margin within
0.1 caliber of 1.5), 💥 Overkill (T/W>15), 🪂 Stuck the Landing (descent <5 m/s).

## 8. UI Spec

- **Builder:** component editors (dropdowns + numeric fields), live SVG side-profile of
  the rocket, instant mass/CG/CP/static-margin readout, warnings shown inline, and
  Save/Load (JSON) + Launch.
- **Dashboard:** grade banner + radar chart + badges + push-pull list of which subsystem
  is strongest/weakest; plots: altitude–time, velocity–time, Mach–time, dynamic
  pressure–q–time, static margin–time (with 1–2 caliber band), thrust curve, and a
  side-profile trajectory arc. Warnings: instability, overstable, high landing speed,
  max-Q / structural, servo-zone T/W flags.
- **Tutorial:** music-of-the-missions using the presets, each with objective +
  step-by-step: (1) launch Sparky & read the grade/plots; (2) fix a nose-weighted stable
  Sparky; (3) push Redline through Mach 1 & find max-Q; (4) tip the fins and watch
  stability fall; (5) fly Icarus II through staging; (6) launch The Lawn Dart and watch
  it tumble. Each mission links into the builder with a pre-filled design + a "check my
  answer" objective check.
- **Units:** SI/Imperial toggle at the top; all UI inputs/outputs render in the chosen
  unit system; the API stays SI.

## 9. Error Handling & Failure Modes

- Input validation at the API edge: non-positive masses/lengths, zero fins with
  motor-size mismatch, unrecoverable designs, motor not mounted, etc. Return 400 with a
  human-readable message.
- Runtime warnings bundled into `FlightResult.warnings`: CP<CG during flight,
  static-margin outside band, landing speed too high, T/W outside safe band,
  structural overload, Mach-transonic dwell warnings.
- Impossible physics guarded (e.g. empty propellant by burnout → coast; landing
  terminates integration).
- Grading is defined for any valid flight, including failures.

## 10. Testing

- `test_atmosphere.py` — ISA ρ/T/P/a vs. reference tables across all layers.
- `test_motor.py` — thrust interpolation, Isp/v_e/ṁ identities, class boundaries.
- `test_aerodynamics.py` — Cd build-up vs. Mach; transonic rise; Reynolds regime;
  nose-shape ranking; Barrowman CN/CP on a known fin set.
- `test_dynamics.py` — no-drag vertical ascent sanity; dt-convergence (<0.1% apogee
  change); mass depletion matches cargo propellant; energy monotonicity in coast.
- `test_stability.py` — CG shift during burn; margin crossing detection.
- `test_design.py` — component mass/CG rollups; preset integrity (every preset must be
  physically buildable and launchable).
- `test_efficiency.py` — sub-score math, weights, grade boundaries, badges.
- **`test_regression_c6.py`** — "Sparky" with a real C6 thrust curve flies to a peak
  altitude within the published ballpark for that motor class (the pipeline sanity check
  the whole project hangs on).
- `test_units.py` — SI↔Imperial conversions round-trip.

All tests run from a venv via `pytest`. No external services required.

## 11. Build Order

1. Repo scaffold, venv, pytest harness, package skeleton.
2. `physics/atmosphere.py` + reference-table tests.
3. `physics/motor.py` + thrust-curve interpolation/mass-flow tests.
4. `physics/dynamics.py` RK4 + no-drag vertical sanity test.
5. `physics/aerodynamics.py` (Cd build-up + Barrowman) + tests.
6. `physics/stability.py` + burn-phase CG tracking + tests.
7. `physics/recovery.py` chute/descent + landing-speed warnings.
8. `design/*` components/rocket/presets → all presets build & fly.
9. `design/efficiency.py` scoring + badges + tests.
10. Multistage staging pass (Icarus II) + tests.
11. `util/units.py` + tests.
12. Web server (`http.server`) + JSON API + validation.
13. Frontend: builder + SVG profile + launch + dashboard + charts + radar.
14. Tutorial missions + README.
15. `test_regression_c6.py` end-to-end; full `pytest` green.
16. Final local run check of the web UI; git history cleaned; push to
    `https://github.com/jabari2breezy/Rocket`.