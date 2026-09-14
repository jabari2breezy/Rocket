const MISSIONS = [
  { preset: "sparky", criterion: "apogee_gt_100",
    num: "01", title: "Build Sparky",
    objective: "Sparky is the friendly intro rocket. Load it, launch it, and watch the flight traces come alive. Every mission ends with a grade, a radar, and a landing card.",
    hint: "Pass when apogee clears 100 m. Sparky cruises to ~145 m." },
  { preset: "sparky", criterion: "score_gte_C",
    num: "02", title: "Read the grade & radar",
    objective: "The debrief grades the design S–F from five weighted sub-scores. Read the radar bars — where does Sparky give up altitude? That bar is your tuning dial.",
    hint: "Pass on grade C or better. Sparky earns a C-grade lesson in drag." },
  { preset: "sparky", criterion: "margin_in_1_2",
    num: "03", title: "Stabilize with nose weight",
    objective: "Static margin = (CP − CG) / d. The healthy band is 1–2 calibers. Add nose ballast and watch the margin line tilt up on the Stability chart.",
    hint: "Pass while margin sits inside 1–2 cal. Sparky starts at ~1.55." },
  { preset: "redline", criterion: "structural_failure",
    num: "04", title: "Push release pressure",
    objective: "Redline flirts with high thrust — and 280 g of release pressure says not today. High-energy rockets can out-thrust their structure. Watch it break up and read the q-limit math.",
    hint: "Pass when the airframe fails. The failure card tells you the exact q and g at breakup." },
  { preset: "icarus", criterion: "was_staged",
    num: "05", title: "Stage Icarus II",
    objective: "Icarus II is a two-stage sounding rocket. The upper stage ignites at altitude and stability is recomputed mid-flight. Watch the second boost on the trajectory arc.",
    hint: "Pass when a stage 2 event is logged." },
  { preset: "lawn-dart", criterion: "unstable",
    num: "06", title: "Launch The Lawn Dart",
    objective: "A very good lesson in what not to launch. Tiny fins and no ballast put CP ahead of CG. Launch it and watch the alpha trace go haywire.",
    hint: "Pass when the flight is flagged unstable — the rocket chooses chaos." }
];

const CRITERIA = {
  apogee_gt_100: f => ({ pass: f.apogee > 100, text: "Apogee " + fmtN(f.apogee, 0) + " m (needs > 100)" }),
  score_gte_C: f => ({ pass: ["A", "B", "C", "S"].includes(f.grade), text: "Grade " + f.grade + " / " + f.overall }),
  margin_in_1_2: f => ({ pass: f.min_margin >= 1 && f.min_margin <= 2, text: "Min margin " + f.min_margin.toFixed(2) + " cal (need 1–2)" }),
  structural_failure: f => ({ pass: !!f.failure, text: f.failure ? ("Failed: " + f.failure.message) : "Still flying (f=" + "no failure" + ")" }),
  was_staged: f => ({ pass: !!(f.stage_events && f.stage_events.length), text: f.stage_events && f.stage_events.length
      ? f.stage_events.map(e => "t+" + fmtN(e.t, 1) + "s @ " + fmtN(e.y, 0) + " m").join(" · ") : "No staging observed" }),
  unstable: f => ({ pass: f.unstable, text: "Unstable: " + f.unstable + " · max alpha " + fmtN(f.max_alpha_deg, 0) + "°" })
};

function runFlight(preset) {
  return fetch("/api/flight/run", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ preset: preset, overrides: {} })
  }).then(r => r.json());
}

function applyCriterionScore(card, mission, result) {
  const sc = result.score || {};
  const f = { ...result.flight, grade: sc.grade, overall: sc.overall, failure: result.failure };
  const check = CRITERIA[mission.criterion](f);
  const ans = card.querySelector(".answer");
  ans.classList.remove("pass", "fail");
  ans.classList.add(check.pass ? "pass" : "fail");
  ans.innerHTML = (check.pass ? "✓ PASS — " : "✗ RETRY — ") + check.text
    + '<br><small>' + (mission.hint || "") + "</small>";
}

function buildMissions() {
  const wrap = document.getElementById("missions");
  wrap.innerHTML = MISSIONS.map(m => `
    <article class="mission" data-preset="${m.preset}" data-criterion="${m.criterion}">
      <span class="m-num">${m.num}</span>
      <h3>${m.title}</h3>
      <p>${m.objective}</p>
      <div class="missions-actions">
        <button class="load">Load into Build Bay</button>
        <button class="check">Check my answer</button>
      </div>
      <p class="answer"></p>
    </article>`).join("");
  wrap.querySelectorAll(".mission").forEach((card, i) => {
    const m = MISSIONS[i];
    card.querySelector(".load").onclick = () => {
      try { localStorage.setItem("rocketlab_preset", card.dataset.preset); } catch (e) {}
      window.location.href = "/";
    };
    card.querySelector(".check").onclick = async () => {
      const b = card.querySelector(".check");
      b.disabled = true; b.textContent = "Checking…";
      const res = await runFlight(card.dataset.preset);
      b.disabled = false; b.textContent = "Check my answer";
      if (!res || !res.ok) {
        const ans = card.querySelector(".answer");
        ans.classList.add("fail");
        ans.textContent = "✗ RETRY — " + ((res && res.errors) || ["sim failed"]).join(", ");
        return;
      }
      applyCriterionScore(card, m, res.result);
    };
  });
}

document.addEventListener("DOMContentLoaded", buildMissions);