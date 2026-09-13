const $ = id => document.getElementById(id);
let preset = 'sparky', design = {}, charts = [], lastResult = null;
const fields = ['length', 'fin_span', 'ballast', 'wind'];

function showValues() {
  fields.forEach(k => {
    const v = Number($(k).value);
    $(k + 'Out').textContent = v.toFixed(k === 'wind' ? 0 : 3)
      + (k === 'length' ? ' m' : k === 'fin_span' ? ' m' : k === 'ballast' ? ' kg' : ' m/s');
  });
}
function load(d) {
  design = d;
  $('designName').textContent = d.name;
  $('nose').value = d.nose;
  fields.forEach(k => $(k).value = d[k]);
  showValues();
}

const TGB = new Set();

function makeChart(id, cfg) {
  const src = $(id);
  const existing = Chart.getChart(src);
  if (existing) existing.destroy();
  const base = {
    type: cfg.type || 'line',
    data: cfg.data,
    options: Object.assign({
      animation: false, maintainAspectRatio: false,
      plugins: { legend: { display: false } },
      scales: {
        x: { ticks: { maxTicksLimit: 6, font: { size: 9 } }, grid: { display: false } },
        y: { ticks: { font: { size: 9 } }, grid: { color: '#edf0f2' } }
      }
    }, cfg.options)
  };
  const c = new Chart(src, base);
  charts.push(c);
  return c;
}

async function init() {
  const p = await fetch('/api/presets').then(r => r.json());
  const nav = $('presets');
  Object.entries(p).forEach(([key, d]) => {
    const b = document.createElement('button');
    b.textContent = d.name;
    b.onclick = () => {
      preset = key;
      [...nav.children].forEach(x => x.classList.remove('active'));
      b.classList.add('active');
      load(d);
    };
    if (key === 'sparky') b.classList.add('active');
    nav.append(b);
  });
  load(p.sparky);
}
fields.forEach(k => $(k).oninput = showValues);
$('nose').onchange = e => design.nose = e.target.value;

function setAlert(kind, html) {
  const a = $('alert');
  a.classList.remove('hidden', 'ok', 'ng');
  a.classList.add(kind || 'ng');
  a.innerHTML = html;
  a.classList.add('on');
}
function clearAlert() { $('alert').classList.add('hidden'); }

function drawFlight(s) {
  const lab = s.map(p => p.t.toFixed(2));
  makeChart('flightChart', {
    data: { labels: lab, datasets: [
      { data: s.map(p => p.y), borderColor: '#ff6535', pointRadius: 0, borderWidth: 2 },
      { data: s.map(p => p.v * 5), borderColor: '#2476a8', pointRadius: 0, borderWidth: 1.5 }
    ] }
  });
}
function drawDrag(s) {
  const lab = s.map(p => p.t.toFixed(2));
  makeChart('dragChart', {
    data: { labels: lab, datasets: [
      { data: s.map(p => p.q / 1000), borderColor: '#ffc837', pointRadius: 0, borderWidth: 2, yAxisID: 'y' },
      { data: s.map(p => p.mach), borderColor: '#112f47', pointRadius: 0, borderWidth: 1.5, yAxisID: 'y2' },
      { data: s.map(p => p.stagnation_temp / 100), borderColor: '#e34329', pointRadius: 0, borderWidth: 1.5, borderDash: [4, 3], yAxisID: 'y2' }
    ] },
    options: { scales: {
      y: { position: 'left', title: { display: true, text: 'kPa', font: { size: 9 } }, ticks: { font: { size: 9 } }, grid: { color: '#edf0f2' } },
      y2: { position: 'right', min: 0, ticks: { font: { size: 9 } }, grid: { drawOnChartArea: false } }
    } }
  });
}
function drawMargin(s) {
  const lab = s.map(p => p.t.toFixed(2));
  const f = lastResult.flight;
  let low = null, up = null;
  const row = s.map(p => p.static_margin);
  if (row.length) {
    low = Math.min.apply(null, row), up = Math.max.apply(null, row);
    if (up < 1) up = 1;
    if (low > 2) low = 2;
  }
  const band = { lo: low === null ? 1 : low, hi: up === null ? 2 : up };
  const ds = [
    { data: s.map(() => band.lo), fill: { target: 1, above: 'rgba(63,150,240,.10)' }, backgroundColor: 'rgba(63,150,240,.10)', borderWidth: 0, pointRadius: 0 },
    { data: s.map(() => band.hi), fill: false, borderWidth: 0, pointRadius: 0 },
    { data: s.map(() => 1.5), borderColor: '#59c6e7', borderWidth: 1, borderDash: [6, 4], pointRadius: 0 },
    { data: s.map(p => p.static_margin), borderColor: '#ff5d3a', pointRadius: 0, borderWidth: 2 }
  ];
  makeChart('marginChart', {
    data: { labels: lab, datasets: ds },
    options: { scales: { y: { suggestedMin: 0, suggestedMax: 3, ticks: { font: { size: 9 } }, grid: { color: '#edf0f2' } } } }
  });
  const cls = f.min_margin < 1 ? 'unstable' : f.min_margin > 2 ? 'overstable' : 'stable';
  $('marginNote').textContent = cls.toUpperCase() + ' · min ' + f.min_margin.toFixed(2) + ' cal';
}
function drawTrajectory(s) {
  makeChart('trajChart', {
    type: 'scatter',
    data: { datasets: [{
      data: s.map(p => ({ x: p.x, y: p.y })),
      borderColor: '#ff5d3a', backgroundColor: '#ff5d3a', pointRadius: 0, borderWidth: 2, showLine: true
    }] },
    options: { scales: {
      x: { title: { display: true, text: 'drift (m)', font: { size: 9 } }, ticks: { font: { size: 9 } }, grid: { color: '#edf0f2' } },
      y: { title: { display: true, text: 'altitude (m)', font: { size: 9 } }, ticks: { font: { size: 9 } }, grid: { color: '#edf0f2' } }
    } }
  });
}
function drawRadar(subscores) {
  $('radar').innerHTML = Object.entries(subscores).map(([k, v]) =>
    `<div><span>${k}</span><span class="bar"><i style="width:${v}%"></i></span><b>${v}</b></div>`).join('');
}
function drawOutcome(f) {
  const rec = { recovered: { c: 'ok', t: 'RECOVERED' }, serviceable: { c: '', t: 'SERVICEABLE' },
                damaged: { c: 'ng', t: 'DAMAGED' }, destroyed: { c: 'ng', t: 'DESTROYED' } }[f.recovery_status] || { c: '', t: f.recovery_status || '—' };
  const events = (f.stage_events && f.stage_events.length) ? `<em>${f.stage_events.join(' · ')}</em>` : '';
  $('outcome').innerHTML = `<div class="outcome-card ${rec.c}"><span class="oc-tag">${rec.t}</span>
    <b>${spd(f.landing_speed, 1)} landing</b>
    <small>${dz(f.downrange_drift, 1)} downrange · t+${f.flight_time.toFixed(1)} s</small>${events}</div>`;
}
function renderResult(r) {
  const s = r.series, f = r.flight;
  const warn = r.warnings || [];
  const fail = r.failure;
  charts.forEach(c => c.destroy()); charts = [];
  drawFlight(s); drawDrag(s); drawMargin(s); drawTrajectory(s);
  $('maxQ').textContent = 'MAX-Q ' + psu(f.max_q, 0);
  if (fail) {
    setAlert('ng', `<b>⚠ ${fail}</b>`);
    $('phase').textContent = 'Flight aborted';
  } else if (warn.length) {
    setAlert('ok', `<b>⚠ ${warn.join(' · ')}</b>`);
  } else {
    clearAlert();
  }
  $('warning').textContent = f.unstable ? '⚠ UNSTABLE: CP has caught CG. Expect a lawn dart.'
    : f.overstable ? '⚠ Overstable: wind will steal altitude.'
    : '✓ Stable through burn — send it.';
  $('warning').style.color = f.unstable ? '#e34329' : (f.overstable ? '#e3a929' : '#27844f');
  const sc = r.score;
  if (sc) {
    $('grade').innerHTML = sc.grade + '<small>' + sc.overall + '/100</small>';
    $('verdict').textContent = f.unstable ? 'The rocket chose chaos.' : 'That was a clean flight.';
    $('details').textContent = f.unstable
      ? 'Your center of pressure sits too close to (or ahead of) center of gravity. Add nose ballast or move bigger fins aft.'
      : `Apogee ${dz(f.apogee, 0)}. Max Mach ${f.max_mach.toFixed(2)} at q = ${psu(f.max_q, 0)}. ${sc.explanation}`;
    $('badges').innerHTML = sc.badges.map(x => `<span class="badge">${x[0]} ${x[1]}</span>`).join('')
      || '<span class="badge">Keep tuning — badges await.</span>';
    drawRadar(sc.subscores);
  }
  drawOutcome(f);
  $('results').classList.remove('hidden');
}
function refreshLast() {
  if (!lastResult) return;
  const f = lastResult.flight;
  $('maxQ').textContent = 'MAX-Q ' + psu(f.max_q, 0);
  renderResult(lastResult);
}
$('unitsToggle').onclick = () => {
  units = units === 'SI' ? 'IMPERIAL' : 'SI';
  $('unitsToggle').textContent = units;
  refreshLast();
};

$('launch').onclick = async () => {
  const overrides = { nose: $('nose').value };
  fields.forEach(k => overrides[k] = Number($(k).value));
  $('launch').disabled = true;
  $('launch').textContent = 'IGNITING…';
  clearAlert();
  const res = await fetch('/api/flight/run', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ preset, overrides })
  }).then(r => r.json()).catch(() => null);
  $('launch').disabled = false;
  $('launch').innerHTML = 'LAUNCH AGAIN <span>↗</span>';
  if (!res || !res.ok) {
    setAlert('ng', '<b>⚠ ' + ((res && res.errors) || ['Simulation failed.']).join(' · ') + '</b>');
    return;
  }
  const result = res.result;
  lastResult = result;
  $('phase').textContent = 'Flight in progress';
  animate(result.series);
  renderResult(result);
};

function animate(series) {
  let i = 0;
  const maxY = Math.max.apply(null, series.map(p => p.y));
  const timer = setInterval(() => {
    const p = series[Math.min(i, series.length - 1)];
    const ratio = Math.min(.82, Math.max(0, (p.y / Math.max(maxY, 1e-9)) * .8));
    $('alt').textContent = dz(p.y, 0);
    $('mach').textContent = 'M ' + p.mach.toFixed(2);
    $('flyer').style.bottom = (38 + ratio * 300) + 'px';
    $('flyer').style.left = (16 + ratio * 24) + '%';
    $('trail').style.height = (ratio * 250) + 'px';
    if (++i >= series.length) {
      clearInterval(timer);
      $('phase').textContent = 'Flight complete';
    }
  }, 8);
}
init();