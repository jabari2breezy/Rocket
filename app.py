import json
from copy import deepcopy
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from design.presets import PRESETS
from physics.dynamics import simulate

ROOT = Path(__file__).parent

class RocketHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs): super().__init__(*args, directory=str(ROOT / "ui" / "static"), **kwargs)
    def _json(self, value):
        payload=json.dumps(value).encode(); self.send_response(200); self.send_header("Content-Type","application/json"); self.send_header("Content-Length",str(len(payload))); self.end_headers(); self.wfile.write(payload)
    def do_GET(self):
        if self.path == "/api/presets":
            return self._json({k:{key:value for key,value in v.items() if key != "curve"} for k,v in PRESETS.items()})
        if self.path == "/": self.path = "/index.html"
        return super().do_GET()
    def do_POST(self):
        if self.path != "/api/simulate": self.send_error(404); return
        try: incoming=json.loads(self.rfile.read(int(self.headers.get("Content-Length",0))))
        except json.JSONDecodeError: self.send_error(400,"Invalid JSON"); return
        design=deepcopy(PRESETS.get(incoming.get("preset","sparky"), PRESETS["sparky"]))
        for key in ("nose","length","diameter","dry_mass","prop_mass","fin_span","fin_count","fin_position","ballast","chute_diameter","wind"):
            if key in incoming: design[key]=incoming[key]
        self._json({"design":design,"result":simulate(design)})

if __name__ == "__main__":
    print("Rocket Lab ready at http://localhost:5050")
    ThreadingHTTPServer(("127.0.0.1",5050),RocketHandler).serve_forever()
