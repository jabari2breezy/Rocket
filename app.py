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
        errors = _validate(design)
        if self.path == "/api/design/validate":
            return self._json({"ok": not errors, "errors": errors})
        if errors:
            return self._json({"ok": False, "errors": errors}, 400)
        return self._json({"ok": True, "design": _strip_curve(design),
                           "result": simulate(design)})


if __name__ == "__main__":
    print("Rocket Lab ready at http://localhost:5050")
    ThreadingHTTPServer(("127.0.0.1", 5050), RocketHandler).serve_forever()