var units = "SI";
const UNITS = {
  SI:       { l:"m", s:"m/s", m:"kg", a:"m²", p:"Pa",  t:"K" },
  IMPERIAL: { l:"ft", s:"mph", m:"lb", a:"ft²", p:"psi", t:"°F" }
};
function unitSet(){ return UNITS[units] || UNITS.SI; }
function fmtN(v, d){ return (v === null || v === undefined || !isFinite(v)) ? "—" : (+v).toFixed(d); }
function altU(v){ const u = unitSet(); return u.l === "ft" ? v / 0.3048 : v; }
function velU(v){ const u = unitSet(); return u.s === "mph" ? v * 2.2369362921 : v; }
function masU(v){ const u = unitSet(); return u.m === "lb" ? v * 2.2046226218 : v; }
function preU(v){ const u = unitSet(); return u.p === "psi" ? v / 6894.757293168 : v; }
function temU(v){ const u = unitSet(); return u.t === "°F" ? (v * 9 / 5 - 459.67) : v; }
function dz(v, d){ return fmtN(altU(v), d) + " " + unitSet().l; }
function spd(v, d){ return fmtN(velU(v), d) + " " + unitSet().s; }
function msw(v, d){ return fmtN(masU(v), d) + " " + unitSet().m; }
function psu(v, d){ return fmtN(preU(v), d) + " " + unitSet().p; }
function kts(v, d){ return fmtN(temU(v), d) + " " + unitSet().t; }