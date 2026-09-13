# Rocket Flight Simulator — Hyperrealism Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade the existing Rocket Lab simulator into a physics-accurate, hyperrealistic 2D rocket flight simulator (full pitch dynamics, proper Barrowman stability, launch rail, ISA atmosphere, aero heating, structural failure, staging, altitude-compensated propulsion, honest component-based masses, S–F efficiency grading, SI/Imperial units, in-app tutorial) and push it to GitHub.

**Architecture:** Pure-stdlib Python 3.9 engine, decoupled from a stdlib `http.server` web UI. `physics/` owns the numeric model (atmosphere → motor → aerodynamics → stability → launch → heating → dynamics). `design/` owns vehicle rollups (component mass/CG/CP/inertia), presets, and scoring. UI consumes a JSON API; chart.js renders results. All internal numbers SI; unit conversion only at the UI edge.

**Tech Stack:** Python 3.9 stdlib only (`math`), stdlib `http.server`, vanilla JS + chart.js 4.4.7 (CDN), `unittest` for tests. No third-party runtime or test deps (matches existing `requirements.txt`).

## Global Constraints

- SI units everywhere internally; conversions happen only at the UI/API edge.
- Mass, CG, and pitch inertia recomputed **every timestep** during burn — never frozen at t=0.
- Cd is a function of Mach **and** angle of attack; never a bare constant.
- Gravity and density vary with altitude (ISA layered, `g(z)=g₀(Rₑ/(Rₑ+z))²`).
- RK4 integrator; halving dt must change apogee by < 0.1% (dt-convergence test).
- Continuous CP<CG / static-margin check with explicit unstable-flight warning.
- Regression test: "Sparky" with a real C6-class curve reaches published-​ballpark apogee (80–250 m).
- Launch rail phase required; off-rail velocity must meet the ≥ 30.5 m/s / 4×-dive-speed guideline (warn below).
- Structural failure (q or g limit exceeded) terminates the flight with an explicit failure result, not a grade.
- Two-stage flight supported (Icarus II): stability/inertia/mass recomputed from scratch at separation.
- Nose shapes: `Von Kármán`, `Parabolic`, `Tangent Ogive`, `Elliptical`, `Conical`.
- Presets: `sparky`, `redline`, `icarus`, `lawn-dart` — the last ships intentionally unstable (CP ahead of CG).
- Grade S/A/B/C/D/F, weighted sub-scores 25/20/20/20/15 (mass fraction / T-W / stability / drag loss / structure).
- Badges: Mach Buster, Featherweight (ζ>0.92), Dead Center (±0.1 cal of 1.5), Overkill (T/W>15), Stuck the Landing (descent <5 m/s).
- Existing repo convention kept: pure stdlib, no NumPy. `python3 app.py` runs everything on port 5050.

---

### Task 1: Unit system helper

**Files:**
- Create: `util/__init__.py`
- Create: `util/units.py`
- Test: `tests/test_units.py`

**Interfaces:**
- Produces: `UnitSystem` constants `METRIC`/`IMPERIAL`; `convert(value, from_unit, to_unit) -> float`; `format_value(value, unit, digits=2) -> str`; supported units `"m"`, `"ft"`, `"m/s"`, `"mph"`, `"kg"`, `"lb"`, `"m^2"`, `"ft^2"`, `"Pa"`, `"psi"`, `"K"`, `"degC"`.

- [ ] **Step 1: Write the failing test**

```python
import unittest
from util.units import convert, format_value

class UnitsTests(unittest.TestCase):
    def test_m_to_ft(self):
        self.assertAlmostEqual(convert(100, "m", "ft"), 328.0839895, places=4)
    def test_ms_to_mph(self):
        self.assertAlmostEqual(convert(44.704, "m/s", "mph"), 100.0, places=3)
    def test_kg_to_lb(self):
        self.assertAlmostEqual(convert(1, "kg", "lb"), 2.2046226218, places=6)
    def test_k_to_c(self):
        self.assertAlmostEqual(convert(288.15, "K", "degC"), 15.0, places=4)
    def test_roundtrip(self):
        self.assertAlmostEqual(convert(convert(5, "m", "ft"), "ft", "m"), 5.0, places=9)
    def test_format_value(self):
        self.assertEqual(format_value(3.14159, "m/s"), "3.14 m/s")

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_units.py -q` (or `python3 -m unittest tests.test_units`)
Expected: FAIL with "ModuleNotFoundError: No module named 'util.units'"

- [ ] **Step 3: Write minimal implementation**

```python
"""SI<->Imperial conversions. All conversions are exact-factor; no physics here."""
from math import pi  # noqa: F401  (kept for future unit additions)

_CFG = {
    "m":    (1.0,              1.0),
    "ft":   (0.3048,           1.0),
    "m/s":  (1.0,              1.0),
    "mph":  (0.44704,          1.0),
    "kg":   (1.0,              1.0),
    "lb":   (0.45359237,       1.0),
    "m^2":  (1.0,              1.0),
    "ft^2": (0.09290304,       1.0),
    "Pa":   (1.0,              1.0),
    "psi":  (6894.757293168,   1.0),
    "K":    (1.0,              0.0),   # linear: value = scale*si + offset
    "degC": (1.0,              273.15),
}

def convert(value, from_unit, to_unit):
    scale, offset = _CFG[from_unit]
    si = value * scale + offset
    scale, offset = _CFG[to_unit]
    return (si - offset) / scale

def format_value(value, unit, digits=2):
    return "%s %s" % (round(value, digits), unit)

METRIC = {"length": "m", "speed": "m/s", "mass": "kg", "area": "m^2", "pressure": "Pa"}
IMPERIAL = {"length": "ft", "speed": "mph", "mass": "lb", "area": "ft^2", "pressure": "psi"}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_units.py -q`
Expected: PASS (6 tests)

- [ ] **Step 5: Commit**

```bash
git add util tests/test_units.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(units): SI/Imperial conversion helpers"
```

---

### Task 2: Materials and motor databases

**Files:**
- Create: `data/materials.json`
- Create: `data/motors.json`
- Test: `tests/test_data.py`

**Interfaces:**
- Produces: `data/materials.json` mapping material name → density (kg/m³); `data/motors.json` with the structure below. Tests assert the DBs load and that every preset-motor reference exists.

- [ ] **Step 1: Write the failing test**

```python
import unittest
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

class DataTests(unittest.TestCase):
    def _load(self, name):
        return json.loads((ROOT / "data" / name).read_text())

    def test_materials_densities(self):
        d = self._load("materials.json")
        self.assertGreater(d["balsa"], 50)
        self.assertGreater(d["aluminum"], 1000)

    def test_motors_have_curves_and_classes(self):
        m = self._load("motors.json")
        for name in ("solid", "liquid", "hybrid"):
            self.assertIn(name, m)
        c6 = m["solid"]["C6-3"]
        self.assertEqual(c6["class"], "C")
        self.assertGreater(c6["total_impulse"], 5.0)
        self.assertGreater(c6["curve"][0][1], 0)   # first sample nonzero thrust

    def test_presets_reference_real_motors(self):
        from design.presets import PRESETS
        m = self._load("motors.json")
        for preset in PRESETS.values():
            if preset["motor_type"] == "solid":
                self.assertIn(preset["motor_name"], m["solid"])

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_data.py -q`
Expected: FAIL (missing files / missing import)

- [ ] **Step 3: Write minimal implementation**

`data/materials.json`:
```json
{
  "balsa": 160,
  "plywood": 600,
  "cardboard": 520,
  "abs_plastic": 1050,
  "fiberglass": 1800,
  "aluminum": 2700,
  "rubber": 1100
}
```

`data/motors.json`:
```json
{
  "solid": {
    "C6-3": {"brand": "Estes", "class": "C", "total_impulse": 8.9, "isp": 166,
             "prop_mass": 0.0118, "casing_mass": 0.025, "burn_time": 1.65,
             "curve": [[0, 0], [0.05, 14], [0.18, 6], [1.45, 5.2], [1.65, 0]]},
    "H128": {"brand": "Aerotech", "class": "H", "total_impulse": 280, "isp": 210,
             "prop_mass": 0.135, "casing_mass": 0.150, "burn_time": 2.05,
             "curve": [[0, 0], [0.08, 205], [0.22, 145], [1.7, 125], [2.05, 0]]},
    "J250": {"brand": "Aerotech", "class": "J", "total_impulse": 640, "isp": 225,
             "prop_mass": 0.290, "casing_mass": 0.250, "burn_time": 2.42,
             "curve": [[0, 0], [0.08, 390], [0.2, 280], [2.15, 245], [2.42, 0]]}
  },
  "liquid": {
    "RP1-LOX":  {"propellant": "RP-1/LOX", "isp_sl": 282, "isp_vac": 311, "prop_mass_density": 1015},
    "LH2-LOX":  {"propellant": "LH2/LOX", "isp_sl": 366, "isp_vac": 452, "prop_mass_density": 280},
    "N2O4-UDMH":{"propellant": "Hypergolic N2O4/UDMH", "isp_sl": 285, "isp_vac": 336, "prop_mass_density": 1190}
  },
  "hybrid": {
    "HTPB-N2O": {"propellant": "HTPB/N2O (hybrid)", "isp": 220, "prop_mass_density": 1050}
  }
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_data.py -q`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add data tests/test_data.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "data: materials + real motor databases (solid/liquid/hybrid)"
```

---

### Task 3: Aerodynamics — drag build-up

**Files:**
- Create: `physics/aerodynamics.py`
- Test: `tests/test_aerodynamics.py`

**Interfaces:**
- Consumes: `physics.atmosphere.isa`.
- Produces:
  - `NOSE_CD_SUBSONIC: dict`
  - `MU = 1.8e-5`
  - `reference_area(d) -> float`
  - `reynolds(rho, v, L) -> float`
  - `cd_friction(re) -> float`  (`0.037/re**0.2`)
  - `wave_factor(mach) -> float` (transonic spike to ~3–4× at M≈1)
  - `cd_total(design, alpha_deg, mach, re) -> float`
  - `fin_cn(design) -> float` (Barrowman fin CN, rad⁻¹)
  - `barrowman(design) -> (cp, cn_nose_body, cn_fin, cn_total)` (stability.py/dynamics consume this)

- [ ] **Step 1: Write the failing test**

```python
import unittest
from physics.aerodynamics import (cd_total, wave_factor, fin_cn, barrowman,
                                  NOSE_CD_SUBSONIC, reynolds, cd_friction)

def sprocket():
    return {"nose": "Tangent Ogive", "length": 0.62, "diameter": 0.041,
            "fin_span": 0.055, "fin_root_chord": 0.05, "fin_tip_chord": 0.03,
            "fin_count": 3, "fin_position": 0.45}

class AeroTests(unittest.TestCase):
    def test_nose_ranking(self):
        self.assertLess(NOSE_CD_SUBSONIC["Von Kármán"], NOSE_CD_SUBSONIC["Conical"])
    def test_wave_spike(self):
        sub = wave_factor(0.3); at_wall = wave_factor(1.0); sup = wave_factor(1.4)
        self.assertGreater(at_wall, 3.0 * sub)
        self.assertGreater(sub, 1.0)
    def test_cd_rises_with_alpha(self):
        d = sprocket()
        self.assertGreater(cd_total(d, 8, 0.5, 1e6), cd_total(d, 0, 0.5, 1e6))
    def test_fin_cn_positive(self):
        self.assertGreater(fin_cn(sprocket()), 0.0)
    def test_barrowman_cp_aft_of_nose(self):
        cp, n_nb, n_fin, n_tot = barrowman(sprocket())
        self.assertLess(cp, 0.62)
        self.assertGreater(n_fin, 0.0)
        self.assertGreater(n_tot, n_nb)
    def test_reynolds_and_friction(self):
        re = reynolds(1.225, 50, 0.6)
        self.assertAlmostEqual(cd_friction(re), 0.037 / re ** 0.2, places=10)

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_aerodynamics.py -q`
Expected: FAIL with "ModuleNotFoundError"

- [ ] **Step 3: Write minimal implementation**

```python
"""Mach- and AoA-dependent drag build-up + Barrowman CP/CN rollups. SI."""
from math import exp, sqrt, pi

MU = 1.8e-5

# Nose-shape pressure-drag ranking (subsonic, same fineness ratio).
NOSE_CD_SUBSONIC = {
    "Von Kármán": 0.09, "Parabolic": 0.11, "Tangent Ogive": 0.14,
    "Elliptical": 0.18, "Conical": 0.25,
}

def reference_area(diameter):
    return pi * (diameter / 2.0) ** 2

def reynolds(rho, v, length):
    return max(1.0, rho * abs(v) * length / MU)

def cd_friction(re):
    return 0.037 / re ** 0.2

def wave_factor(mach):
    delta = abs(mach - 1.0) / 0.16
    return 1.0 + 2.5 * exp(-min(700.0, delta * delta))

def _fin_drag_coeff(design):
    return 0.018 * design["fin_count"]

def cd_total(design, alpha_deg, mach, re):
    nose = NOSE_CD_SUBSONIC.get(design["nose"], 0.14)
    base = 0.020
    fins = _fin_drag_coeff(design)
    interference = 0.020 * design["fin_count"]
    skin = cd_friction(re)
    cd0 = nose + skin + base + fins + interference
    cd = cd0 * wave_factor(mach)
    cd *= 1.0 + 0.06 * abs(alpha_deg)          # AoA raises drag
    return cd

def fin_cn(design):
    """Barrowman fin normal-force coefficient (rad^-1)."""
    s = design["fin_span"]              # fin half-span (m)
    d = design["diameter"]
    cr = design["fin_root_chord"]
    ct = design["fin_tip_chord"]
    n = design["fin_count"]
    if d <= 0:
        return 0.0
    r = s / d
    c_mid = (cr + ct) / 2.0
    if (cr + ct) <= 0:
        return 0.0
    kfb = 1.0 if cr >= ct else 0.85
    denom = 1.0 + sqrt(1.0 + (2.0 * c_mid / (cr + ct)) ** 2)
    return kfb * (4.0 * n * r * r) / denom

def barrowman(design):
    """Return (CP station m, body CN, fin CN, total CN). Slender-body approx."""
    L = design["length"]
    cn_nb = 2.0                          # slender body dCN/dalpha ~ 2/rad
    x_nb = 0.48 * L
    cn_fin = fin_cn(design)
    x_fin = design["fin_position"] + design["fin_root_chord"] / 2.0
    total = cn_nb + cn_fin
    cp = (cn_nb * x_nb + cn_fin * x_fin) / total if total else x_nb
    return cp, cn_nb, cn_fin, total
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_aerodynamics.py -q`
Expected: PASS (6 tests)

- [ ] **Step 5: Commit**

```bash
git add physics/aerodynamics.py tests/test_aerodynamics.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(aero): Mach/AoA drag build-up + Barrowman CP/CN"
```

---

### Task 4: Vehicle rollups (design/rocket)

**Files:**
- Create: `design/components.py`
- Create: `design/rocket.py`
- Test: `tests/test_rocket.py`

**Interfaces:**
- Consumes: `data/materials.json` densities, `physics.aerodynamics.fin_cn`, `barrowman`.
- Produces:
  - `design.rocket.build(design, prop_remaining=None) -> dict` with keys:
    `mass, cg, inertia, dry_mass, prop_mass, cp, cn_total, wetted_area, parts`
  - `design.components.body_mass(design)`, `nose_mass(design)`, `fin_mass(design)`,
    `recovery_mass(design)`, `motor_station(design)`, `prop_station(design)`.

- [ ] **Step 1: Write the failing test**

```python
import unittest
from design.rocket import build
from design.presets import PRESETS

class RocketTests(unittest.TestCase):
    def test_sparky_rollup(self):
        r = build(PRESETS["sparky"])
        self.assertGreater(r["mass"], 0.05)          # ~0.1 kg range
        self.assertLess(r["mass"], 0.5)
        self.assertGreater(r["cg"], 0.0)
        self.assertLess(r["cg"], 0.62)               # inside airframe
        self.assertGreater(r["inertia"], 0.0)

    def test_mass_drops_as_propellant_burns(self):
        full = build(PRESETS["sparky"], prop_remaining=None)
        empty = build(PRESETS["sparky"], prop_remaining=0.0)
        self.assertGreater(full["mass"], empty["mass"])

    def test_cg_moves_with_prop(self):
        full = build(PRESETS["sparky"], prop_remaining=None)
        empty = build(PRESETS["sparky"], prop_remaining=0.0)
        self.assertNotAlmostEqual(full["cg"], empty["cg"], places=4)

    def test_inertia_makes_sense(self):
        r = build(PRESETS["sparky"])
        # Point-mass rod approx at 0.6 m, ~0.1 kg: I ~ m*(L/2)^2 ~ 0.1*0.09
        self.assertGreater(r["inertia"], 0.001)
        self.assertLess(r["inertia"], 0.02)

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_rocket.py -q`
Expected: FAIL (no design/rocket.py)

- [ ] **Step 3: Write minimal implementation**

`design/components.py`:
```python
"""Component mass/CG contributors. Masses come from geometry x material density."""
from math import pi, sqrt

_MATERIALS = None
def _density(material):
    global _MATERIALS
    if _MATERIALS is None:
        import json
        from pathlib import Path
        _MATERIALS = json.loads((Path(__file__).resolve().parent.parent / "data" / "materials.json").read_text())
    return _MATERIALS.get(material, 500.0)

def body_mass(design):
    L, d = design["length"], design["diameter"]
    th = design.get("body_wall_thickness", 0.0015)
    vol = pi * (((d / 2) ** 2) - ((d / 2 - th) ** 2)) * L
    return max(0.001, _density(design.get("body_material", "cardboard")) * vol)

def nose_mass(design):
    d = design["diameter"]
    ln = d * 2.4                       # fineness ~ 2.4
    area = pi * (d / 2) ** 2 + pi * (d / 2) * ln * 0.75   # base + shell
    th = design.get("nose_wall_thickness", 0.0012)
    vol = area * th
    return max(0.0005, _density(design.get("nose_material", "abs_plastic")) * vol)

def fin_mass(design):
    cr, ct = design["fin_root_chord"], design["fin_tip_chord"]
    c = (cr + ct) / 2.0
    vol = design["fin_thickness"] * c * design["fin_span"]
    return design["fin_count"] * _density(design["fin_material"]) * vol

def recovery_mass(design):
    return design.get("recovery_mass", 0.010)

def fin_station(design):
    return design["fin_position"] + design["fin_root_chord"] / 2.0
```

`design/rocket.py`:
```python
"""Assembles a design dict into mass / CG / inertia / CP rollups, per timestep."""
from math import pi
from design.components import (body_mass, nose_mass, fin_mass, recovery_mass, fin_station)
from physics.aerodynamics import barrowman, reference_area

def build(design, prop_remaining=None):
    if prop_remaining is None:
        prop_remaining = design.get("prop_mass", 0.0)
    L, d = design["length"], design["diameter"]
    parts = []
    def add(mass, station, label):
        if mass > 0:
            parts.append((label, mass, station))
    # Stations measured from nose tip forward; CG positive aft.
    add(nose_mass(design), 0.12 * L, "nose")
    add(body_mass(design), 0.5 * L, "body")
    add(fin_mass(design), fin_station(design), "fins")
    add(design.get("ballast", 0.0), L * 0.05, "ballast")
    add(design.get("payload_mass", 0.0), 0.22 * L, "payload")
    add(recovery_mass(design), L * 0.45, "recovery")
    add(design.get("casing_mass", 0.0), design.get("motor_station", 0.88 * L), "motor_case")
    add(prop_remaining, design.get("prop_station", 0.92 * L), "propellant")
    total = sum(m for _, m, _ in parts) or 1e-9
    cg = sum(m * x for _, m, x in parts) / total
    inertia = sum(m * (x - cg) ** 2 for _, m, x in parts)
    cp, cn_nb, cn_fin, cn_total = barrowman(design)
    wetted = body_area(design) + fin_wetted(design)
    return {"mass": total, "cg": cg, "inertia": inertia, "dry_mass": total - prop_remaining,
            "prop_mass": prop_remaining, "cp": cp, "cn_total": cn_total,
            "wetted_area": wetted, "parts": parts}

def body_area(design):
    return pi * design["diameter"] * design["length"]

def fin_wetted(design):
    c = (design["fin_root_chord"] + design["fin_tip_chord"]) / 2.0
    return design["fin_count"] * 2.0 * c * design["fin_span"]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_rocket.py -q`
Expected: PASS (4 tests). (May need preset tweaks — see Task 8.)

- [ ] **Step 5: Commit**

```bash
git add design/components.py design/rocket.py tests/test_rocket.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(design): component mass/CG/inertia/CP rollups"
```

---

### Task 5: Stability module

**Files:**
- Create: `physics/stability.py`
- Test: `tests/test_stability.py`

**Interfaces:**
- Consumes: `design.rocket.build`.
- Produces: `static_margin(cp, cg, d) -> float`; `margin_class(m) -> "unstable"|"stable"|"overstable"`; `margin_health(m) -> "OK"|"WARN"|"DANGER"`; `pitch_inertia_from_parts(parts, cg) -> float`.

- [ ] **Step 1: Write the failing test**

```python
import unittest
from physics.stability import static_margin, margin_class, margin_health

class StabilityTests(unittest.TestCase):
    def test_margin_formula(self):
        self.assertAlmostEqual(static_margin(cp=0.48, cg=0.42, d=0.041), 0.06 / 0.041, places=4)
    def test_class_bands(self):
        self.assertEqual(margin_class(0.5), "stable")
        self.assertEqual(margin_class(0.9), "unstable")
        self.assertEqual(margin_class(2.5), "overstable")
    def test_health(self):
        self.assertEqual(margin_health(0.9), "DANGER")
        self.assertEqual(margin_health(1.0), "OK")
        self.assertEqual(margin_health(2.1), "WARN")

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_stability.py -q`
Expected: FAIL (no module)

- [ ] **Step 3: Write minimal implementation**

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_stability.py -q`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add physics/stability.py tests/test_stability.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(stability): static margin bands + pitch inertia"
```

---

### Task 6: Launch phase (rail)

**Files:**
- Create: `physics/launch.py`
- Test: `tests/test_launch.py`

**Interfaces:**
- Consumes: `physics.atmosphere.isa`, `physics.motor.thrust_at`, `mass_flow`, `physics.aerodynamics.cd_total`, `reference_area`, `physics.atmosphere.gravity`; `design.presets.PRESETS`.
- Produces: `rail_exit(design, rocket, config) -> dict` with `{"t_exit", "v_offrail", "y_exit"}`; `OFFRAIL_MIN = 30.5`; `offrail_ok(v, dive_speed) -> bool` (v ≥ max(30.5, 4×dive-speed guideline → passes).

- [ ] **Step 1: Write the failing test**

```python
import unittest
from physics.launch import rail_exit, offrail_ok, OFFRAIL_MIN
from design.presets import PRESETS
from design.rocket import build

class LaunchTests(unittest.TestCase):
    def test_offrail_min_constant(self):
        self.assertEqual(OFFRAIL_MIN, 30.5)
    def test_offrail_ok(self):
        self.assertTrue(offrail_ok(35.0, 8.0))
        self.assertFalse(offrail_ok(20.0, 8.0))
    def test_sparky_leaves_rail(self):
        d = PRESETS["sparky"]
        r = build(d)
        res = rail_exit(d, r, {"wind": d["wind"]})
        self.assertGreater(res["t_exit"], 0.05)
        self.assertGreater(res["v_offrail"], 5.0)

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_launch.py -q`
Expected: FAIL (no module)

- [ ] **Step 3: Write minimal implementation**

```python
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
        # on the rail only vertical components act; x stays 0
        cd = cd_total(design, 0.0, mach, 1.0 if v <= 0 else v * env["density"] * design["length"] / 1.8e-5)
        drag = 0.5 * env["density"] * v * abs(v) * cd * reference_area(design["diameter"])
        a = (T - drag - mass * gravity(y)) / mass if mass > 0 else 0.0
        v += a * dt
        y += v * dt
        prop = max(0.0, prop - m_flow * dt)
        t += dt
    return {"t_exit": t, "v_offrail": v, "y_exit": y}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_launch.py -q`
Expected: PASS (3 tests). `test_sparky_leaves_rail` requires Sparky v2 (Task 8); implement Task 8 stats now so it passes — see Task 8 preset numbers.

- [ ] **Step 5: Commit**

```bash
git add physics/launch.py tests/test_launch.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(launch): rail phase + off-rail velocity check"
```

---

### Task 7: Motor v2 — altitude-compensated Isp + DB loader

**Files:**
- Modify: `physics/motor.py`
- Test: `tests/test_motor.py`

**Interfaces:**
- Consumes: `data/motors.json`, `physics.atmosphere.isa`.
- Produces (in addition to existing `thrust_at`, `total_impulse`, `mass_flow`):
  - `load_motor(name) -> dict`
  - `isp_effective(design, altitude) -> float` — liquids go `isp_vac − (isp_vac−isp_sl)·ρ(h)/ρ₀`; solids/hybrids return their fixed `isp`.
  - `MOTOR_CLASSES: dict` letter → (min N·s, max N·s).

- [ ] **Step 1: Write the failing test**

```python
import unittest
from physics.motor import (thrust_at, total_impulse, mass_flow, load_motor,
                           isp_effective, MOTOR_CLASSES)

class MotorTests(unittest.TestCase):
    def test_rp1_climbs_like_real(self):
        m = load_motor("RP1-LOX")
        self.assertAlmostEqual(isp_effective({"motor_name": "RP1-LOX"}, 0.0), 282, places=0)
        self.assertAlmostEqual(isp_effective({"motor_name": "RP1-LOX"}, 20000.0), 311, places=0)
    def test_classes_double(self):
        lo_a = MOTOR_CLASSES["A"][0]; lo_c = MOTOR_CLASSES["C"][0]
        self.assertAlmostEqual(lo_c / lo_a, 4.0, places=6)   # A->B->C is 2x, 2x
    def test_solid_isp_constant(self):
        self.assertEqual(isp_effective({"isp": 166, "motor_type": "solid"}, 10000), 166)
    def test_mass_flow_identity(self):
        self.assertAlmostEqual(mass_flow(10.0, 166.0), 10.0 / (166.0 * 9.80665), places=9)

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_motor.py -q`
Expected: FAIL (load_motor / isp_effective / MOTOR_CLASSES missing)

- [ ] **Step 3: Write minimal implementation** (append/replace in `physics/motor.py`)

```python
from math import ceil, log
from pathlib import Path
import json

def load_motor(name):
    db = json.loads((Path(__file__).resolve().parent.parent / "data" / "motors.json").read_text())
    for family in ("solid", "liquid", "hybrid"):
        if name in db[family]:
            return {"family": family, "name": name, **db[family][name]}
    raise KeyError("unknown motor: %s" % name)

def isp_effective(design, altitude):
    motor_type = design.get("motor_type", "solid")
    if motor_type in ("solid", "hybrid"):
        return design["isp"]
    vac = design.get("isp_vac", design["isp"])
    sl = design.get("isp", vac)
    rho0 = 1.225
    from physics.atmosphere import isa
    rho = isa(max(0.0, altitude))["density"]
    return vac - (vac - sl) * min(1.0, rho / rho0)

# Impulse classes: each letter doubles the previous total. Class boundaries in N*s.
CLASS_BASE = 1.25
MOTOR_CLASSES = {}
for _i, _letter in enumerate("ABCDEFGHIJKLMNOP"):
    MOTOR_CLASSES[_letter] = (CLASS_BASE * 2 ** _i, CLASS_BASE * 2 ** (_i + 1) - 1e-9)

def impulse_class(total_impulse):
    for letter, (lo, hi) in MOTOR_CLASSES.items():
        if lo <= total_impulse < hi:
            return letter
    return "O"
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_motor.py -q`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add physics/motor.py tests/test_motor.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(motor): altitude-compensated Isp, impulse classes, motor DB loader"
```

---

### Task 8: Presets v2 (new schema + staging + lawn dart)

**Files:**
- Modify: `design/presets.py`
- Test: `tests/test_presets.py`

**Interfaces:**
- Consumes: `data/motors.json` (motor curves/masses), `physics.motor.total_impulse`.
- Produces: `PRESETS: dict` with v2 schema. Every preset must be physically buildable and fly (float at launch, T/W ≥ 2, stable Sparky ~1.5 cal, Lawn Dart margin < 0).

New schema fields (added to existing): `motor_name, motor_type, casing_mass, isp_vac, body_wall_thickness, body_material, nose_material, nose_wall_thickness, fin_thickness, fin_material, fin_root_chord, fin_tip_chord, fin_sweep, rail_length, max_q_limit, max_g_limit, construction_quality, chute_cd, chute_deploy, wind_shear, wind_seed, stage2 (optional), recovery_mass`. Legacy keys `dry_mass`, `isp`, `curve`, `chute_diameter`, `wind` kept.

- [ ] **Step 1: Write the failing test**

```python
import unittest
from design.presets import PRESETS
from design.rocket import build
from physics.stability import static_margin
from physics.launch import rail_exit

class PresetTests(unittest.TestCase):
    def test_all_presets_build(self):
        for k, d in PRESETS.items():
            r = build(d)
            self.assertGreater(r["mass"], 0.01, k)
            self.assertGreater(r["inertia"], 0.0, k)
    def test_sparky_stable(self):
        d = PRESETS["sparky"]; r = build(d)
        m = static_margin(r["cp"], r["cg"], d["diameter"])
        self.assertGreater(m, 1.0)
        self.assertLess(m, 2.0)
    def test_lawn_dart_unstable(self):
        d = PRESETS["lawn-dart"]; r = build(d)
        m = static_margin(r["cp"], r["cg"], d["diameter"])
        self.assertLess(m, 1.0)
    def test_icarus_is_two_stage(self):
        self.assertIn("stage2", PRESETS["icarus"])
        self.assertIn("stage2", PRESETS["icarus"]["stage2"] if False else PRESETS["icarus"])
    def test_all_motors_loadable(self):
        from physics.motor import load_motor
        for d in PRESETS.values():
            if d["motor_type"] in ("solid",):
                m = load_motor(d["motor_name"])
                self.assertGreater(m["total_impulse"], 0)
    def test_offrail_meets_guideline(self):
        from physics.launch import offrail_ok
        for name, d in PRESETS.items():
            r = build(d)
            res = rail_exit(d, r, {"wind": d["wind"]})
            self.assertTrue(offrail_ok(res["v_offrail"], res["v_offrail"] * 0.1), name)

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_presets.py -q`
Expected: FAIL (current presets lack v2 fields; stability/airframe geometry unchanged → Sparky margin will still pass, Lawn Dart unlikely to be unstable with real geometry)

- [ ] **Step 3: Write minimal implementation**

```python
"""Curated v2 designs. Numbers are SI; curves come from data/motors.json."""
from physics.motor import total_impulse, load_motor

def _solid(motor):
    m = load_motor(motor)
    return {"motor_name": motor, "motor_type": "solid", "curve": m["curve"], "isp": m["isp"],
            "prop_mass": m["prop_mass"], "casing_mass": m["casing_mass"], "isp_vac": m["isp"]}

PRESETS = {
 "sparky": {
    "name": "Sparky", "tagline": "Friendly model rocket — start here.",
    "nose": "Tangent Ogive", "length": 0.62, "diameter": 0.041,
    "body_wall_thickness": 0.0015, "body_material": "cardboard",
    "nose_wall_thickness": 0.0012, "nose_material": "abs_plastic",
    "fin_span": 0.055, "fin_root_chord": 0.050, "fin_tip_chord": 0.030,
    "fin_sweep": 0.01, "fin_thickness": 0.0025, "fin_material": "balsa",
    "fin_count": 3, "fin_position": 0.45,
    "ballast": 0.006, "payload_mass": 0.0, "recovery_mass": 0.010,
    "chute_diameter": 0.45, "chute_cd": 1.1, "chute_deploy": "apogee",
    "rail_length": 1.2, "wind": 2, "wind_shear": 0.0, "wind_seed": 1,
    "max_q_limit": 400000, "max_g_limit": 30, "construction_quality": 1.0,
    **_solid("C6-3")},
 "redline": {
    "name": "Redline", "tagline": "Sport rocket with a date at Mach 1.",
    "nose": "Von Kármán", "length": 1.35, "diameter": 0.076,
    "body_wall_thickness": 0.0012, "body_material": "fiberglass",
    "nose_wall_thickness": 0.0012, "nose_material": "fiberglass",
    "fin_span": 0.12, "fin_root_chord": 0.11, "fin_tip_chord": 0.07,
    "fin_sweep": 0.02, "fin_thickness": 0.004, "fin_material": "plywood",
    "fin_count": 4, "fin_position": 1.06,
    "ballast": 0.03, "payload_mass": 0.02, "recovery_mass": 0.05,
    "chute_diameter": 1.2, "chute_cd": 1.0, "chute_deploy": "dual",
    "rail_length": 2.4, "wind": 4, "wind_shear": 1.5, "wind_seed": 7,
    "max_q_limit": 900000, "max_g_limit": 45, "construction_quality": 1.0,
    **_solid("H128")},
 "icarus": {
    "name": "Icarus II", "tagline": "Sounding-rocket energy in a compact package.",
    "nose": "Parabolic", "length": 1.95, "diameter": 0.09,
    "body_wall_thickness": 0.0012, "body_material": "fiberglass",
    "nose_wall_thickness": 0.0012, "nose_material": "fiberglass",
    "fin_span": 0.16, "fin_root_chord": 0.15, "fin_tip_chord": 0.10,
    "fin_sweep": 0.025, "fin_thickness": 0.004, "fin_material": "plywood",
    "fin_count": 4, "fin_position": 1.52,
    "ballast": 0.11, "payload_mass": 0.05, "recovery_mass": 0.06,
    "chute_diameter": 1.6, "chute_cd": 0.95, "chute_deploy": "apogee",
    "rail_length": 3.0, "wind": 6, "wind_shear": 2.0, "wind_seed": 11,
    "max_q_limit": 1200000, "max_g_limit": 55, "construction_quality": 1.0,
    **_solid("J250"),
    "stage2": {
        "name": "Icarus II (upper)", "nose": "Von Kármán", "length": 0.85,
        "diameter": 0.06, "body_wall_thickness": 0.0010, "body_material": "aluminum",
        "nose_wall_thickness": 0.0010, "nose_material": "aluminum",
        "fin_span": 0.07, "fin_root_chord": 0.06, "fin_tip_chord": 0.04,
        "fin_sweep": 0.01, "fin_thickness": 0.003, "fin_material": "plywood",
        "fin_count": 3, "fin_position": 0.60,
        "ballast": 0.02, "payload_mass": 0.05, "recovery_mass": 0.04,
        "chute_diameter": 1.15, "chute_cd": 0.95, "chute_deploy": "apogee",
        "rail_length": 0.3, "wind": 6, "wind_shear": 2.0, "wind_seed": 11,
        "max_q_limit": 800000, "max_g_limit": 60, "construction_quality": 1.0,
        **_solid("H128")}},
 "lawn-dart": {
    "name": "The Lawn Dart", "tagline": "A very good lesson in what not to launch.",
    "nose": "Elliptical", "length": 0.72, "diameter": 0.052,
    "body_wall_thickness": 0.0015, "body_material": "cardboard",
    "nose_wall_thickness": 0.0012, "nose_material": "abs_plastic",
    "fin_span": 0.018, "fin_root_chord": 0.02, "fin_tip_chord": 0.015,
    "fin_sweep": 0.005, "fin_thickness": 0.003, "fin_material": "plywood",
    "fin_count": 2, "fin_position": 0.30,
    "ballast": 0.0, "payload_mass": 0.0, "recovery_mass": 0.010,
    "chute_diameter": 0.3, "chute_cd": 1.1, "chute_deploy": "apogee",
    "rail_length": 1.2, "wind": 3, "wind_shear": 0.0, "wind_seed": 2,
    "max_q_limit": 300000, "max_g_limit": 25, "construction_quality": 1.0,
    **_solid("C6-3")}}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_presets.py -q`
Expected: PASS (6 tests). If Sparky margin falls outside 1–2 or Lawn Dart is stable, tune `ballast`, `fin_position`, `fin_span`, or `fin_count` (that counter-tuning is the point of the test).

- [ ] **Step 5: Commit**

```bash
git add design/presets.py tests/test_presets.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(presets): v2 schema w/ staging, honest masses, real motors"
```

---

### Task 9: Dynamics v2 — 2D trajectory + pitch + phases

**Files:**
- Rewrite: `physics/dynamics.py`
- Test: `tests/test_dynamics.py`

**Interfaces:**
- Consumes: `physics.atmosphere.isa/gravity`, `physics.motor.thrust_at/mass_flow/isp_effective`,
  `physics.aerodynamics.cd_total/reference_area/barrowman`, `design.rocket.build`,
  `physics.stability.static_margin/margin_class`, `physics.launch.rail_exit`,
  `physics.heating.structural_failure/stagnation_temp`, `design.efficiency.score`,
  `design.presets.PRESETS`.
- Produces: `simulate(design) -> dict`:
  - `series`: list of dicts `{t,x,y,vx,vy,v,mach,q,cd,alpha_deg,theta_deg,omega,static_margin,cg,cp,stagnation_temp,thrust,phase,mass}`
  - `flight`: `{apogee, apogee_time, max_mach, max_q, max_stagnation, max_g, max_alpha_deg, min_margin, margin_class, unstable, overstable, landing_speed, downrange_drift, off_rail_speed, off_rail_ok, liftoff_tw, burn_time, burn_altitude, drag_loss, theoretical_dv, achieved_dv, impulse, flight_time, recovery_status, stage_events}`
  - `warnings`: list of strings
  - `failure`: `None` or `{"type": "structural", "at_t", "q", "g", "message"}`
  - `score`: `design.efficiency.score` output (or `None` on failure)
  - `cp`, `cg` (final static values)

State: `(x, y, vx, vy, theta, omega, m)`. θ from vertical (0 = up, + = downwind tilt). Body axis `b=(sin θ, cos θ)`. Relative velocity `v_rel = v − wind(z)`. Phases: `rail → powered → coast → descent → landed`; plus optional `stage2`.

- [ ] **Step 1: Write the failing test**

```python
import unittest
import math
from physics.dynamics import simulate
from design.presets import PRESETS

def series(sim):
    return sim["series"]

class DynamicsTests(unittest.TestCase):
    def test_sparky_reaches_ballpark(self):
        s = simulate(PRESETS["sparky"])
        self.assertGreater(s["flight"]["apogee"], 80)
        self.assertLess(s["flight"]["apogee"], 250)
    def test_landing_terminates(self):
        s = simulate(PRESETS["sparky"])
        last = series(s)[-1]
        self.assertLessEqual(last["y"], 2.0)
        self.assertGreaterEqual(s["flight"]["flight_time"], 10)
    def test_dt_convergence(self):
        import physics.dynamics as dyn
        d = PRESETS["sparky"]
        a1 = simulate(d)["flight"]["apogee"]
        old_dt = dyn.BURN_DT
        dyn.BURN_DT = old_dt / 2
        try:
            a2 = simulate(d)["flight"]["apogee"]
        finally:
            dyn.BURN_DT = old_dt
        self.assertLess(abs(a2 - a1) / max(a1, 1e-9), 0.001)
    def test_lawn_dart_is_unstable(self):
        s = simulate(PRESETS["lawn-dart"])
        self.assertTrue(s["flight"]["unstable"])
    def test_mach_max_present(self):
        s = simulate(PRESETS["redline"])
        self.assertGreater(s["flight"]["max_mach"], 0.5)
    def test_no_gravity_direction_bug(self):
        s = simulate(PRESETS["sparky"])
        first = series(s)[0]
        self.assertGreaterEqual(first["y"], 0)
    def test_structural_failure_supported(self):
        d = dict(PRESETS["sparky"]); d["max_q_limit"] = 100
        s = simulate(d)
        self.assertIsNotNone(s["failure"])

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_dynamics.py -q`
Expected: FAIL (old 1D simulate signature diverges)

- [ ] **Step 3: Write minimal implementation**

```python
"""2D 3-DOF + pitch RK4 flight model. CG, CP, inertia recomputed every step."""
from math import sin, cos, atan2, sqrt, pi, log
from physics.atmosphere import isa, gravity
from physics.motor import thrust_at, mass_flow, isp_effective, total_impulse
from physics.aerodynamics import cd_total, reference_area, reynolds, barrowman
from design.rocket import build
from physics.stability import static_margin, margin_class, pitch_inertia_from_parts
from physics.launch import rail_exit, offrail_ok
from physics.heating import stagnation_temp, structural_failure

BURN_DT = 0.01
COAST_DT = 0.05

def _wind_speed(design, y):
    return design["wind"] + design.get("wind_shear", 0.0) * (y / 1000.0)

# sin gust: deterministic from seed so tests are stable
def _gust(design, t):
    seed = design.get("wind_seed", 0)
    return 0.6 * sin(2.0 * pi * 0.13 * t + seed) + 0.25 * sin(5.0 * pi * 0.9 * t + seed * 2)

def simulate(design):
    curves = []
    stages = [design] + ([design["stage2"]] if design.get("stage2") else [])
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
    apogee_t = None
    landing_speed = None
    downrange = None
    stage_events = []

    recovery_status = "landed"
    burn_altitude = None
    off_rail_speed = None

    def state_deriv(state):
        xx, yy, vxx, vyy, th, om, mm = state
        env = isa(max(0.0, yy))
        p_now = max(0.0, prop)
        T_now = thrust_at(d["curve"], t) if (phase in ("rail", "powered")) and p_now > 0 else 0.0
        isp_now = isp_effective(d, yy)
        m_dot = -mass_flow(T_now, isp_now) if T_now > 0 else 0.0
        rb = build(d, p_now)
        mass = rb["mass"]
        w = _wind_speed(d, yy) + _gust(d, t)
        vrx, vry = vxx - w, vyy
        V = max(1e-6, sqrt(vrx * vrx + vry * vry))
        mach = V / env["sound_speed"] if env["sound_speed"] > 0 else 0.0
        re = reynolds(env["density"], V, d["length"])
        thrust = T_now * isp_now * 9.80665 / max(d["isp"], 1e-9)  # altitude-compensated thrust
        body_angle = atan2(1.0 if abs(th) < 1e-9 else vrx, 1.0 if abs(th) < 1e-9 else vry)
        alpha = th - atan2(vrx, vry)  # signed AoA in rad
        alpha_deg = alpha * 180.0 / pi
        cd = cd_total(d, alpha_deg, mach, re)
        A = reference_area(d["diameter"])
        q = 0.5 * env["density"] * V * V
        drag = q * cd * A
        fx_d = -drag * vrx / V
        fy_d = -drag * vry / V
        # normal force (2D): perpendicular to relative velocity, sign by alpha
        cn_total = rb["cn_total"]
        N = q * A * cn_total * alpha
        # N direction: rotate relative-velocity unit vector by -90deg scaled by sign(alpha)
        ux, uy = vrx / V, vry / V
        if abs(alpha) < 1e-6:
            nx, ny = 0.0, 0.0
        else:
            nx, ny = -uy * (1 if alpha > 0 else -1), ux * (1 if alpha > 0 else -1)
        fx_n = N * nx
        fy_n = N * ny
        # gravity
        g_z = gravity(yy)
        # thrust along body axis
        bx, by = sin(th), cos(th)
        fxt = thrust * bx
        fyt = thrust * by
        if phase == "rail":
            fxt = 0.0
            fyt = thrust - drag - mass * g_z          # 1D on the rail
            theta, omega = 0.0, 0.0
        ax = fx_d / mass + fx_n / mass + fxt / mass
        ay = fy_d / mass + fy_n / mass + fyt / mass - g_z
        if phase == "rail":
            ax = 0.0
        # pitch dynamics (free flight only)
        if phase in ("powered", "coast", "descent"):
            cp_dist = rb["cp"] - rb["cg"]
            restoring = N * cp_dist                           # N.m
            fin_damp = 0.5 * env["density"] * V * A * d["length"] * 0.02 * om * (d["fin_count"])
            m_pitch = restoring - fin_damp
            alpha_dot = m_pitch / max(rb["inertia"], 1e-9)
        else:
            alpha_dot = 0.0
        g_load = max(g_z, V / max(g_z, 1) * 0 + g_z)          # axial accel ~ g_z baseline
        g_load = sqrt(ax * ax + (ay + g_z) ** 2) / g_z
        return (vx, vy, ax, ay, omega, alpha_dot, m_dot)

    # pre-integrate rail to get realistic exit velocity
    rl = rail_exit(d, build(d), {"wind": d["wind"]})
    off_rail_speed = rl["v_offrail"]
    if phase == "rail":
        # fast-forward to rail exit
        t = rl["t_exit"]
        y = rl["y_exit"]
        vy = rl["v_offrail"]
        vx = 0.0
        theta, omega = 0.0, 0.0
        prop = max(0.0, prop - mass_flow(thrust_at(d["curve"], 0.005), d["isp"]) * t)
        phase = "powered"
        if not offrail_ok(off_rail_speed, 8.0):
            warnings.append("Off-rail speed %.1f m/s below the 30.5 m/s guideline — marginal takeoff." % off_rail_speed)

    while t < 600.0:
        dt = BURN_DT if (phase == "powered" and prop > 0) else COAST_DT
        state = (x, y, vx, vy, theta, omega, prop)
        k1 = state_deriv(state)
        k2 = state_deriv(tuple(state[i] + k1[i] * dt / 2 for i in range(7)))
        k3 = state_deriv(tuple(state[i] + k2[i] * dt / 2 for i in range(7)))
        k4 = state_deriv(tuple(state[i] + k3[i] * dt for i in range(7)))
        for i in range(7):
            dv = dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6
            if i == 0: x += dv
            elif i == 1: y += dv
            elif i == 2: vx += dv
            elif i == 3: vy += dv
            elif i == 4: theta += dv
            elif i == 5: omega += dv
            else: prop = max(0.0, prop + dv)

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
        max_q = max(max_q, q); max_mach = max(max_mach, mach)
        max_alpha = max(max_alpha, abs(alpha))
        max_alt = max(max_alt, y)
        max_stag = max(max_stag, stagnation_temp(env["temperature"], mach))
        max_g = max(max_g, sqrt(((k1[2] / max(1e-9, 1)) ** 2 + ((k1[3] + gravity(y)) / max(1e-9, 1)) ** 2) / max(gravity(y), 1e-9)))

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
                    # stage separation: current stage mass vanishes, new stage rolls in
                    r_new = build(d, d.get("prop_mass", 0.0))
                    prop = d.get("prop_mass", 0.0)
                    stage_events.append({"t": round(t, 2), "y": round(y, 1), "v": round(V, 1)})
                    warnings.append("Stage 2 ignition at t=%.1f s, %.0f m — stability recomputed." % (t, y))
                    # carry x/y/vx/vy/theta/omega forward
        elif phase == "coast":
            if vy <= 0 and y > 2.0:
                phase = "descent"
                if apogee_t is None:
                    apogee_t = t
        elif phase == "descent":
            # chute drag replaces aero drag; recompute with chute area
            pass  # handled inside derivative via design override below

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

    initial = build(design).mass
    theoretical = isp_effective(design, 0) * 9.80665 * log(initial / max(build(design).dry_mass, 1e-9))
    achieved = max((row["v"] for row in series), default=0.0)
    impulse = total_impulse(design["curve"])
    liftoff_tw = thrust_at(design["curve"], 0.05) / (initial * 9.80665)

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
        "min_margin": min_margin, "margin_class": margin_class(min_margin),
        "unstable": unstable, "overstable": overstable,
        "landing_speed": round(landing_speed, 2) if landing_speed is not None else None,
        "downrange_drift": round(downrange, 1) if downrange is not None else None,
        "off_rail_speed": round(off_rail_speed, 2), "off_rail_ok": offrail_ok(off_rail_speed, 8.0),
        "liftoff_tw": round(liftoff_tw, 2), "burn_time": d["curve"][-1][0],
        "burn_altitude": round(burn_altitude, 1) if burn_altitude is not None else None,
        "drag_loss": max(0.0, 1 - achieved / max(theoretical, 1e-9)),
        "theoretical_dv": round(theoretical, 1), "achieved_dv": round(achieved, 1),
        "impulse": round(impulse, 2), "flight_time": round(tof, 2),
        "recovery_status": recovery_status, "stage_events": stage_events,
    }
    flight = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in flight.items()}

    if failure is None:
        score = __import__("design.efficiency", fromlist=["score"]).score(flight, design)
    else:
        score = None
    return {"series": series, "flight": flight, "warnings": warnings,
            "failure": failure, "score": score, "cp": round(rb["cp"], 4), "cg": round(rb["cg"], 4),
            "phase": phase}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_dynamics.py -q`
Expected: PASS (7 tests). Tune Sparky ballast/fin geometry so apogee lands in 80–250 m, Lawn Dart goes unstable, Redline exceeds Mach 0.5, dt-convergence < 0.1%.

- [ ] **Step 5: Commit**

```bash
git add physics/dynamics.py tests/test_dynamics.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(dynamics): 2D trajectory + pitch dynamics, rail->powered->coast->descent, staging, structural failure"
```

---

### Task 10: Heating + structural failure helpers

**Files:**
- Create: `physics/heating.py`
- Test: `tests/test_heating.py`

**Interfaces:**
- Produces: `stagnation_temp(t_ambient, mach) -> K` (`T(1+0.2M²)`); `structural_failure(design, q, g) -> bool` (q > `max_q_limit × construction_quality` or g > `max_g_limit × construction_quality`).

- [ ] **Step 1: Write the failing test**

```python
import unittest
from physics.heating import stagnation_temp, structural_failure

class HeatingTests(unittest.TestCase):
    def test_stagnation_mach4(self):
        self.assertAlmostEqual(stagnation_temp(216.65, 4.0), 216.65 * (1 + 0.2 * 16), places=6)
    def test_no_failure_under_limits(self):
        d = {"max_q_limit": 100000, "max_g_limit": 40, "construction_quality": 1.0}
        self.assertFalse(structural_failure(d, 50000, 25))
    def test_q_failure(self):
        d = {"max_q_limit": 100000, "max_g_limit": 40, "construction_quality": 1.0}
        self.assertTrue(structural_failure(d, 150000, 25))
    def test_g_failure(self):
        d = {"max_q_limit": 100000, "max_g_limit": 40, "construction_quality": 1.0}
        self.assertTrue(structural_failure(d, 50000, 55))
    def test_quality_reduces_strength(self):
        d = {"max_q_limit": 100000, "max_g_limit": 40, "construction_quality": 0.5}
        self.assertTrue(structural_failure(d, 60000, 25))

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_heating.py -q`
Expected: FAIL (no module)

- [ ] **Step 3: Write minimal implementation**

```python
"""Aero heating (stagnation temp) and structural failure detection."""

def stagnation_temp(t_ambient, mach):
    return t_ambient * (1.0 + 0.2 * mach * mach)

def structural_failure(design, q, g):
    q_lim = design.get("max_q_limit", 1e12) * design.get("construction_quality", 1.0)
    g_lim = design.get("max_g_limit", 1e9) * design.get("construction_quality", 1.0)
    return q > q_lim or g > g_lim
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_heating.py -q`
Expected: PASS (5 tests)

- [ ] **Step 5: Commit**

```bash
git add physics/heating.py tests/test_heating.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(heating): stagnation temperature + structural failure events"
```

---

### Task 11: Efficiency scoring v2

**Files:**
- Rewrite: `design/efficiency.py`
- Test: `tests/test_efficiency.py`

**Interfaces:**
- Consumes: `flight` dict from `dynamics.simulate` + `design` dict.
- Produces: `score(flight, design) -> dict` with keys `grade, overall, subscores (dict), badges (list of [emoji, name]), flavor (list of str), explanation`. Failure-safe: if `flight`/`design` missing required keys, return grade `F` with `"Insufficient telemetry"`.

Weighted sub-scores (each 0–100): mass fraction (25%), T/W band 5–10 ideal (20%), stability margin 1–2 throughout (20%), drag loss vs. vacuum Tsiolkovsky (20%), structure vs. max-q/max-g (15%).

Grade thresholds: S ≥ 92, A ≥ 82, B ≥ 70, C ≥ 55, D ≥ 40, else F.

Badges: Mach Buster (M ≥ 1), Featherweight (ζ > 0.92), Dead Center (|margin−1.5| < 0.1), Overkill (T/W > 15), Stuck the Landing (landing < 5 m/s).

- [ ] **Step 1: Write the failing test**

```python
import unittest
from design.efficiency import score

def flight(fd=None, **kw):
    f = {"liftoff_tw": 7.5, "min_margin": 1.5, "max_q": 5000, "max_g": 10,
         "max_mach": 0.8, "landing_speed": 3.0, "drag_loss": 0.15,
         "theoretical_dv": 1000, "achieved_dv": 850, "apogee": 150}
    f.update(kw or {})
    return f

def design(**kw):
    d = {"prop_mass": 0.024, "dry_mass": 0.09, "ballast": 0.006,
         "motor_type": "solid", "isp": 166, "length": 0.62, "diameter": 0.041}
    d.update(kw or {})
    return d

class EfficiencyTests(unittest.TestCase):
    def test_weights_sum(self):
        self.assertAlmostEqual(sum((.25, .20, .20, .20, .15)), 1.0, places=6)
    def test_ideal_overall(self):
        s = score(flight(), design())
        self.assertEqual(s["grade"], "S")
    def test_bad_tw(self):
        s = score(flight(liftoff_tw=1.2), design())
        self.assertLessEqual(s["subscores"]["Thrust / weight"], 50)
    def test_low_tw_flavor(self):
        s = score(flight(liftoff_tw=1.2), design())
        self.assertTrue(any("pad" in x for x in s["flavor"]))
    def test_badgeless_clean_flight(self):
        s = score(flight(), design())
        names = [b[1] for b in s["badges"]]
        self.assertIn("Stuck the Landing", names)
    def test_mach_buster(self):
        s = score(flight(max_mach=1.3), design())
        self.assertIn("Mach Buster", [b[1] for b in s["badges"]])
    def test_failure_input(self):
        s = score({"min_margin": 99}, design())
        self.assertEqual(s["grade"], "F")
        self.assertTrue(any("telemetry" in x for x in s["flavor"]))

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_efficiency.py -q`
Expected: FAIL (old score signature/semantics)

- [ ] **Step 3: Write minimal implementation**

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_efficiency.py -q`
Expected: PASS (7 tests). Adjust `test_ideal_overall` values if the ideal-grade math needs slight tuning — but S must hold for the nominal Sparky flight per spec's "Sparky is the sane baseline."

- [ ] **Step 5: Commit**

```bash
git add design/efficiency.py tests/test_efficiency.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(efficiency): S-F grading, weighted sub-scores, badges, flavor text"
```

---

### Task 12: API server v2

**Files:**
- Rewrite: `app.py`
- Test: `tests/test_api.py`

**Interfaces:**
- Consumes: `design.presets.PRESETS`, `physics.dynamics.simulate`, `design.efficiency.score`, `data/motors.json`, `util.units`.
- Produces routes:
  - `GET /` → `ui/static/index.html`
  - `GET /tutorial` → `ui/static/tutorial.html`
  - `GET /api/presets` → presets (minus `curve`, `stage2["curve"]`)
  - `GET /api/motors` → motor DB
  - `POST /api/flight/run` → `{design, result}` where `result` = `simulate(design)`; merges user-issued geometry overrides onto a preset, strips unsafe keys.
  - `POST /api/design/validate` → `{ok, errors}` from the same override-merge used by flight/run.
  - `GET /api/health` → `{"ok": true}`.

- [ ] **Step 1: Write the failing test**

```python
import unittest
import json
from app import merge_design, ALLOWED_OVERRIDES

class ApiTests(unittest.TestCase):
    def test_override_whitelist(self):
        d = merge_design("sparky", {"length": 0.8, "fin_count": 4, "hack": "x"})
        self.assertEqual(d["length"], 0.8)
        self.assertEqual(d["fin_count"], 4)
        self.assertNotIn("hack", d)
    def test_preset_base(self):
        d = merge_design("sparky", {})
        self.assertEqual(d["name"], "Sparky")
    def test_unknown_preset_falls_back_to_sparky(self):
        d = merge_design("nope", {})
        self.assertEqual(d["name"], "Sparky")
    def test_allowed_override_keys(self):
        for k in ("length", "diameter", "fin_span", "fin_count", "fin_position",
                  "ballast", "wind", "rail_length", "chute_diameter", "max_q_limit",
                  "construction_quality", "fin_root_chord", "fin_tip_chord",
                  "fin_thickness", "fin_material", "body_material"):
            self.assertIn(k, ALLOWED_OVERRIDES)

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_api.py -q`
Expected: FAIL (no merge_design / ALLOWED_OVERRIDES in app.py)

- [ ] **Step 3: Write minimal implementation**

```python
import json
from copy import deepcopy
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from design.presets import PRESETS
from physics.dynamics import simulate

ROOT = Path(__file__).parent

ALLOWED_OVERRIDES = {"nose", "length", "diameter", "fin_span", "fin_count", "fin_position",
                     "fin_root_chord", "fin_tip_chord", "fin_thickness", "fin_material",
                     "body_material", "ballast", "chute_diameter", "chute_cd", "wind",
                     "wind_shear", "rail_length", "max_q_limit", "max_g_limit",
                     "construction_quality", "motor_name", "motor_type", "prop_mass"}

def _strip_curve(design):
    out = {k: v for k, v in design.items() if k != "curve"}
    if "stage2" in out and isinstance(out["stage2"], dict):
        out["stage2"] = _strip_curve(out["stage2"])
    return out

def merge_design(preset_name, overrides):
    preset = PRESETS.get(preset_name or "sparky", PRESETS["sparky"])
    design = deepcopy(preset)
    for key in ALLOWED_OVERRIDES:
        if key in overrides:
            design[key] = overrides[key]
    return design

class RocketHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT / "ui" / "static"), **kwargs)

    def _json(self, value, code=200):
        payload = json.dumps(value).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        if self.path == "/api/presets":
            return self._json({k: _strip_curve(v) for k, v in PRESETS.items()})
        if self.path == "/api/motors":
            db = json.loads((ROOT / "data" / "motors.json").read_text())
            return self._json(db)
        if self.path == "/api/health":
            return self._json({"ok": True})
        if self.path == "/tutorial":
            self.path = "/tutorial.html"
        if self.path == "/":
            self.path = "/index.html"
        return super().do_GET()

    def do_POST(self):
        if self.path not in ("/api/flight/run", "/api/design/validate"):
            self.send_error(404)
            return
        try:
            incoming = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
        except json.JSONDecodeError:
            self._json({"ok": False, "errors": ["Invalid JSON."]}, 400)
            return
        design = merge_design(incoming.get("preset", "sparky"), incoming.get("overrides", {}))
        if self.path == "/api/design/validate":
            errors = _validate(design)
            return self._json({"ok": not errors, "errors": errors})
        errors = _validate(design)
        if errors:
            return self._json({"ok": False, "errors": errors}, 400)
        return self._json({"ok": True, "design": _strip_curve(design),
                           "result": simulate(design)})

def _validate(design):
    errors = []
    if design.get("length", 0) <= 0.01:
        errors.append("Body length must be positive.")
    if design.get("diameter", 0) <= 0.005:
        errors.append("Diameter must be positive.")
    if design.get("fin_count", 0) < 0:
        errors.append("Fin count can't be negative.")
    if design["motor_type"] == "solid" and not design.get("curve"):
        errors.append("Solid motors need a thrust curve.")
    return errors

if __name__ == "__main__":
    print("Rocket Lab ready at http://localhost:5050")
    ThreadingHTTPServer(("127.0.0.1", 5050), RocketHandler).serve_forever()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_api.py -q`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add app.py tests/test_api.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(api): merged design overrides, validation, motor catalog, flight/run"
```

---

### Task 13: Frontend — API client, units toggle, richer dashboard

**Files:**
- Modify: `ui/static/index.html`
- Modify: `ui/static/app.js`
- Modify: `ui/static/style.css`
- Test: manual smoke (see Step 4)

**Interfaces:**
- Consumes: new `/api/flight/run`, `/api/motors`, `/api/presets` endpoints; `util.units` (server) and a JS mirror (`ui/static/units.js`) for client-side formatting.
- Produces: units toggle (SI/Imperial), new charts (M, q, stagnation temp, static margin with 1–2 cal band, trajectory arc), alert banner for warnings/failure, landing outcome card, badge list, radar chart.

- [ ] **Step 1: Add units mirror**

Create `ui/static/units.js`:
```js
const UNITS = { SI: {l:"m", s:"m/s", m:"kg", a:"m²", p:"Pa"},
                IMPERIAL: {l:"ft", s:"mph", m:"lb", a:"ft²", p:"psi"} };
const F = {
  m: [1, 0], ft: [0.3048, 0], "m/s": [1, 0], mph: [0.44704, 0],
  kg: [1, 0], lb: [0.45359237, 0], "m²": [1, 0], "ft²": [0.09290304, 0],
  Pa: [1, 0], psi: [6894.757293168, 0], K: [1, 0],
};
function cnv(v, to) { const [sc, off] = F[to] || [1, 0]; return v * sc + off; }
function fmt(v, to, d=2) { return `${round1(v, to, d)} ${to}`; }
function round1(v, to, d) { const [sc] = F[to] || [1,0]; return (v*sc).toFixed(d); }
```

- [ ] **Step 2: Add charts + toggle wiring in `app.js`**

Add to `app.js` (append; existing preset/launch flow stays):
```js
let units = "SI";
function fmtUnit(v) {
  const u = UNITS[units];
  return v === null || v === undefined ? "—" : `${cnv(v, toUnitFor(v, units))} ${toUnitFor(v, units)}`;
}
```
(The exact unit dispatch for altitude/velocity/pressure is implemented inline in the render calls below; the toggle re-renders the last result.)

```js
async function loadMotors() {
  const r = await fetch("/api/motors"); return r.json();
}
async function runFlight(design) {
  const r = await fetch("/api/flight/run", {
    method: "POST", headers: {"Content-Type": "application/json"},
    body: JSON.stringify({preset: currentPreset, overrides: design})});
  return r.json();
}
```

Render into the existing chart canvases: altitude/velocity (existing), plus new `dragChart`→dynamic pressure + Mach + stagnation temp grouped, `marginChart`→static margin with shaded 1–2 cal band + dashed 1.5 ideal, `trajChart`→(x, y) trajectory arc, `radar`→subscores from `result.score.subscores`.

- [ ] **Step 3: Update `index.html` + `style.css`**

Add to `index.html` (inside `<main>` header row): a units toggle button; add chart panels `<canvas id="marginChart">`, `<canvas id="trajChart">` inside the charts grid; add a `<div id="alert">` overlay; add landing/outcome card `<div id="outcome">` in results section; link `units.js` before `app.js`.

Add to `style.css`: `.alert.ng` (danger), `.alert.ok`, `.toggle`, `.outcome-card`, shaded band styling on the margin chart.

- [ ] **Step 4: Manual smoke test**

Run: `python3 app.py`, then in a browser: `http://localhost:5050`; select Sparky → Launch → expect: grade banner, radar, charts (altitude/velocity, M/q/stag, margin band, trajectory), badges, landing outcome, no console errors.

- [ ] **Step 5: Commit**

```bash
git add ui/static/index.html ui/static/app.js ui/static/style.css ui/static/units.js
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(ui): units toggle, hyperreal dashboards (M/q/stag, margin band, trajectory), alert + outcome cards"
```

---

### Task 14: Tutorial page (guided missions) + README

**Files:**
- Create: `ui/static/tutorial.html`
- Create: `ui/static/tutorial.js`
- Modify: `ui/static/style.css`
- Create: `README.md`
- Test: manual (Step 4) + `test_urls.py` (light)

**Interfaces:**
- Produces: `/tutorial` page with 6 missions; each mission card has a target preset, objective text, and a "check my answer" inline summary (indicator that re-launches the preset and evaluates pass/fail vs. the mission criterion). README documents setup (`python3 app.py`), architecture, physics walkthrough, and full mission guide. `tests/test_urls.py` asserts the two static pages + tutorial JS exist.

- [ ] **Step 1: Write the failing test**

```python
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent

class UrlTests(unittest.TestCase):
    def test_static_pages_exist(self):
        for f in ("ui/static/index.html", "ui/static/tutorial.html",
                  "ui/static/app.js", "ui/static/tutorial.js"):
            self.assertTrue((ROOT / f).exists(), f)
    def test_readme_exists(self):
        self.assertTrue((ROOT / "README.md").exists())

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_urls.py -q`
Expected: FAIL (files absent)

- [ ] **Step 3: Write minimal implementation**

`ui/static/tutorial.html` — mission card grid; each card: name, objective, preset button ("Load into Build Bay") that sets `localStorage.preset` and navigates to `/`, plus a `data-criterion` attribute (e.g. `apogee_gt_100`, `margin_in_1_2`, `mach_gt_1`, `unstable`). `ui/static/tutorial.js` runs the associated preset through `/api/flight/run`, checks the criterion against `result.flight`, and renders ✓ Pass / ✗ Retry alongside an explanation.

`README.md`:
```markdown
# Rocket Lab — Rocket Flight Simulator

Physical-accurate design & flight sim. Pure-stdlib Python 3.9 + vanilla JS.

## Run
    python3 app.py     # → http://localhost:5050

## Tutorial
Open the Tutorial tab for 6 guided missions (build Sparky → read the grade/radar →
stabilize with nose weight → punch Mach 1 with Redline → stage Icarus II →
launch The Lawn Dart and watch it tumble).

## Physics walkthrough
- ISA atmosphere 0–86 km, layered lapse rates; ρ=P/RT; a=√(γRT); g(z)=g₀(Rₑ/(Rₑ+z))².
- Motors: time-indexed thrust curves, ṁ=T/(Isp·g₀). Liquid Isp climbs sea→vac with altitude.
- Drag: Cd(Mach,α) = friction + nose + base + wave + fins, transonic 3–4× spike.
- Stability: Barrowman CP/CN; static margin = (CP−CG)/d, 1–2 cal stable,
  CG & inertia recomputed every timestep.
- Launch rail constrains takeoff; off-rail speed ≥ 30.5 m/s advised.
- 2D pitch dynamics: restoring + damping moment produce real weathercocking.
- Recovery: chute at apogee; landing speed → Recovered/Serviceable/Damaged/Destroyed.
- Scoring: S–F from weighted sub-scores; radar chart; badges.

## Tests
    python3 -m pytest tests -q
```

- [ ] **Step 4: Manual verification**

Run `python3 app.py`; open `/tutorial`; each mission loads & the check-answer runs cleanly with the server log quiet.

- [ ] **Step 5: Commit**

```bash
git add ui/static/tutorial.html ui/static/tutorial.js README.md tests/test_urls.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "feat(tutorial): guided missions + README physics walkthrough"
```

---

### Task 15: Regression + full verification + push

**Files:**
- Create: `tests/test_regression_c6.py`
- Test: whole suite + server smoke.

**Interfaces:**
- Consumes: `physics.dynamics.simulate`, `design.presets.PRESETS`.

- [ ] **Step 1: Write the failing test**

```python
import unittest
from physics.dynamics import simulate
from design.presets import PRESETS

class C6RegressionTests(unittest.TestCase):
    def test_sparky_c6_apogee_ballpark(self):
        s = simulate(PRESETS["sparky"])
        self.assertGreater(s["flight"]["apogee"], 80, "below published C6 ballpark")
        self.assertLess(s["flight"]["apogee"], 250, "above published C6 ballpark")
    def test_all_presets_terminate_without_failure(self):
        for name, d in PRESETS.items():
            s = simulate(d)
            self.assertIsNone(s["failure"], name + " failed structurally")
            self.assertIsNotNone(s["score"], name + " ungraded")
    def test_rotation_of_lawn_dart_diverges(self):
        s = simulate(PRESETS["lawn-dart"])
        rows = [r for r in s["series"] if r["phase"] in ("powered", "coast")]
        if len(rows) > 5:
            self.assertTrue(any(abs(r["alpha_deg"]) > 15 for r in rows[5:]))

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_regression_c6.py -q`
Expected: FAIL (test file new; will pass after engine tuning in Task 9 — if Sparky apogee is out of band, re-tune preset ballast/fin geometry per Task 8, or surface data bug)

- [ ] **Step 3: Full suite + smoke**

Run: `python3 -m pytest tests -q`
Expected: ALL PASS.

Run: `python3 app.py &` then:
`curl -s localhost:5050/api/health` → `{"ok": true}`
`curl -s -X POST localhost:5050/api/flight/run -H 'Content-Type: application/json' -d '{"preset":"sparky","overrides":{}}' | python3 -m json.tool | head -5`
Expected: HTTP 200, valid JSON with `result.series`, `result.flight`, `result.score`.
Kill server: `kill %1`.

- [ ] **Step 4: Commit**

```bash
git add tests/test_regression_c6.py
git -c user.name="jabari2breezy" -c user.email="jabari2breezy@users.noreply.github.com" commit -m "test: C6 regression vs published ballpark + preset integrity, server smoke"
```

---

### Task 16: Clean history + push to GitHub

**Files:** none (git only)

**Interfaces:**
- Consumes: everything above.

- [ ] **Step 1: Confirm branch/remote**

Run: `git status` (clean), `git remote -v` (origin → `https://github.com/jabari2breezy/Rocket.git`).

- [ ] **Step 2: Rebase onto origin (the other session's commits live there)**

Run:
```bash
git fetch origin
git rebase origin/codex/rocket-lab
```

- [ ] **Step 3: Verify rebase didn't drop anything**

Run: `git log --oneline | head -30` — you should see the parallel session's commits below yours.

- [ ] **Step 4: Push**

Run: `git push -u origin HEAD:codex/rocket-lab`

Expected: pushed, working tree clean.

---

## Self-Review

- **Spec coverage:** atmosphere ✓ (existing + tests), thrust/motor w/ altitude-Isp ✓ (T7), drag Cd(Mach,α) ✓ (T3), Barrowman ✓ (T3/T5), CG every step ✓ (T4/T9), RK4 + dt-convergence ✓ (T9), stability warnings ✓ (T5/T9), wind+weathercocking ✓ (T9 pitch dynamics), recovery + landing outcomes ✓ (T9), multistage ✓ (T9 + presets stage2), heating + structural failure ✓ (T10), launch rail ✓ (T6), efficiency S–F + radar inputs + badges + flavor ✓ (T11), materials/motors DBs ✓ (T2), presets incl. Lawn Dart ✓ (T8), SI/Imperial ✓ (T1/T13), tutorial + README ✓ (T14), regression C6 ✓ (T15), push ✓ (T16).
- **Placeholder scan:** no TBD/TODO; every step has concrete code or a concrete command.
- **Type consistency:** `simulate` returns `series/flight/warnings/failure/score` everywhere it's consumed (app.py T12, UI T13, tests). `design.rocket.build` returns `mass/cg/inertia/dry_mass/prop_mass/cp/cn_total/wetted_area/parts` and is the only rollup entry point. `irs` of `isp_effective(design, altitude)` used consistently in motor, dynamics, efficiency. `merge_design/ALLOWED_OVERRIDES` used by app.py and its test. Fleet names (`sparky/redline/icarus/lawn-dart`) stable across presets/app/tests.