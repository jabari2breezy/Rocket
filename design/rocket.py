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