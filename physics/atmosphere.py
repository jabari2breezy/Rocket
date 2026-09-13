"""International Standard Atmosphere (0–86 km), SI units throughout."""
from math import exp, sqrt

G0 = 9.80665
R = 287.05
GAMMA = 1.4
EARTH_RADIUS = 6_371_000.0

# base altitude, base temperature, lapse rate; pressure is calculated layer-to-layer
LAYERS = ((0.0, 288.15, -0.0065), (11000.0, 216.65, 0.0), (20000.0, 216.65, 0.001),
          (32000.0, 228.65, 0.0028), (47000.0, 270.65, 0.0),
          (51000.0, 270.65, -0.0028), (71000.0, 214.65, -0.002))

def gravity(altitude: float) -> float:
    return G0 * (EARTH_RADIUS / (EARTH_RADIUS + max(0.0, altitude))) ** 2

def isa(altitude: float) -> dict:
    """Return temperature K, pressure Pa, density kg/m³ and sound speed m/s."""
    h = max(0.0, min(86000.0, altitude))
    pressure = 101325.0
    for i, (base_h, base_t, lapse) in enumerate(LAYERS):
        next_h = LAYERS[i + 1][0] if i + 1 < len(LAYERS) else h
        if h <= base_h:
            break
        top = min(h, next_h)
        dh = top - base_h
        if lapse:
            t_top = base_t + lapse * dh
            pressure *= (t_top / base_t) ** (-G0 / (lapse * R))
        else:
            pressure *= exp(-G0 * dh / (R * base_t))
        if h <= next_h:
            break
    # temperature comes from the containing layer
    layer = next(row for row in reversed(LAYERS) if h >= row[0])
    temperature = layer[1] + layer[2] * (h - layer[0])
    return {"temperature": temperature, "pressure": pressure, "density": pressure / (R * temperature),
            "sound_speed": sqrt(GAMMA * R * temperature)}
