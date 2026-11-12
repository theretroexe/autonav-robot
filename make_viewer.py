"""Bakes out/robot.glb + out/report.json into one self-contained viewer page: out/robot_viewer.html.
Run after robot.py:  python make_viewer.py
"""
import base64
import json
import os
import re
import robot as R
import make_plates

PAGE = r"""<meta charset="utf-8">
<title>Autonav R1 viewer</title>
<link rel="icon" href="data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A//www.w3.org/2000/svg%22%20viewBox%3D%220%200%2064%2064%22%20role%3D%22img%22%20aria-labelledby%3D%22t%22%3E%3Ctitle%20id%3D%22t%22%3EAutonav%20R1%3C/title%3E%3Crect%20width%3D%2264%22%20height%3D%2264%22%20rx%3D%2214%22%20fill%3D%22%232b2f36%22/%3E%3Cg%20stroke%3D%22%238a93a0%22%20stroke-width%3D%222.4%22%20stroke-linecap%3D%22round%22%20fill%3D%22none%22%3E%3Cpath%20d%3D%22M32%2051V27M17%2051h30M19%2027h26%22/%3E%3C/g%3E%3Cg%20fill%3D%22%23f2f4f6%22%3E%3Crect%20x%3D%2213%22%20y%3D%2244%22%20width%3D%227%22%20height%3D%2214%22%20rx%3D%222.2%22/%3E%3Crect%20x%3D%2244%22%20y%3D%2244%22%20width%3D%227%22%20height%3D%2214%22%20rx%3D%222.2%22/%3E%3Crect%20x%3D%2215.5%22%20y%3D%2220%22%20width%3D%227%22%20height%3D%2214%22%20rx%3D%222.2%22%20transform%3D%22rotate%28-31%2019%2027%29%22/%3E%3Crect%20x%3D%2241.5%22%20y%3D%2220%22%20width%3D%227%22%20height%3D%2214%22%20rx%3D%222.2%22%20transform%3D%22rotate%28-21.5%2045%2027%29%22/%3E%3C/g%3E%3Cpath%20d%3D%22M24.2%209.6a9%209%200%200%201%2015.6%200%22%20stroke%3D%22%23f5821e%22%20stroke-width%3D%222.4%22%20stroke-linecap%3D%22round%22%20fill%3D%22none%22/%3E%3Ccircle%20cx%3D%2232%22%20cy%3D%2215%22%20r%3D%224.2%22%20fill%3D%22%23f5821e%22/%3E%3C/svg%3E">
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@500;700;800&family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<style>
/* layout: full-height workbench - model on the left, inspection sheet on the right; stacks on phones */
:root {
  --bg: #e9ecef; --panel: #f7f8f9; --ink: #161a1f; --muted: #5b6470; --line: #cfd5dc;
  --accent: #d9661a; --ok: #1f7a4d; --warn: #b7791f; --scene: #e3e7eb;
  --f-display: "Archivo", "Helvetica Neue", Arial, sans-serif;
  --f-body: "IBM Plex Sans", "Segoe UI", system-ui, sans-serif;
  --f-mono: "IBM Plex Mono", ui-monospace, Consolas, monospace;
}
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg: #111316; --panel: #171a1e; --ink: #e8ebee; --muted: #9aa3ad; --line: #2a2f35;
  --accent: #f08a3e; --ok: #4cc38a; --warn: #e0b04a; --scene: #15181c; color-scheme: dark } }
:root[data-theme="dark"] {
  --bg: #111316; --panel: #171a1e; --ink: #e8ebee; --muted: #9aa3ad; --line: #2a2f35;
  --accent: #f08a3e; --ok: #4cc38a; --warn: #e0b04a; --scene: #15181c; color-scheme: dark }
html, body { height: 100%; }
body { background: var(--bg); color: var(--ink); font: 14px/1.45 var(--f-body); }
.app { display: flex; height: 100%; min-height: 0; }
.stage { position: relative; flex: 1; min-width: 0; background: var(--scene); }
#cv { position: absolute; inset: 0; width: 100%; height: 100%; display: block; touch-action: none; }
.hud { position: absolute; left: max(16px, env(safe-area-inset-left, 0px)); top: 16px; pointer-events: none; }
.hud h1 { font: 800 clamp(22px, 3vw, 34px)/1 var(--f-display); letter-spacing: -0.01em; margin: 0; text-wrap: balance; }
.hud p { margin: 6px 0 0; color: var(--muted); font-size: 13px; max-width: 46ch; }
.tip { position: absolute; pointer-events: none; font: 500 12px var(--f-mono); background: var(--panel); color: var(--ink);
  border: 1px solid var(--line); padding: 3px 7px; border-radius: 4px; white-space: nowrap; transform: translate(12px, 12px); }
.legend { position: absolute; left: max(16px, env(safe-area-inset-left, 0px)); bottom: 14px; display: flex; gap: 14px; flex-wrap: wrap;
  font: 12px var(--f-mono); color: var(--muted); pointer-events: none; }
.legend i { display: inline-block; width: 10px; height: 10px; border-radius: 2px; margin-right: 6px; vertical-align: -1px; }
aside { width: 360px; flex: none; overflow-y: auto; background: var(--panel); border-left: 1px solid var(--line);
  padding-block: 18px 28px; padding-left: max(18px, env(safe-area-inset-left, 0px)); padding-right: max(18px, env(safe-area-inset-right, 0px));
  display: flex; flex-direction: column; gap: 22px; }
section h2 { font: 700 11px var(--f-display); letter-spacing: 0.12em; text-transform: uppercase; color: var(--muted); margin: 0 0 10px; }
table { width: 100%; border-collapse: collapse; font-size: 13px; }
td { padding: 6px 0; border-bottom: 1px solid var(--line); vertical-align: top; }
td.v { text-align: right; font: 500 12.5px var(--f-mono); font-variant-numeric: tabular-nums; white-space: nowrap; padding-left: 10px; }
.pill { display: inline-block; font: 600 10.5px var(--f-mono); padding: 1px 6px; border-radius: 3px; margin-left: 6px; }
.pass { color: var(--ok); border: 1px solid currentColor; }
.est { color: var(--warn); border: 1px solid currentColor; }
.ctl { display: grid; grid-template-columns: 1fr auto; gap: 6px 10px; align-items: center; }
.ctl label { font-size: 13px; }
.ctl output { font: 500 12.5px var(--f-mono); font-variant-numeric: tabular-nums; }
input[type=range] { grid-column: 1 / -1; width: 100%; accent-color: var(--accent); }
.checks { display: flex; flex-direction: column; gap: 6px; margin-top: 10px; }
.checks label { display: flex; gap: 8px; align-items: center; font-size: 13px; cursor: pointer; }
input[type=checkbox] { accent-color: var(--accent); }
.steer-read { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-top: 10px; }
.steer-read div { border: 1px solid var(--line); border-radius: 4px; padding: 6px 8px; }
.steer-read b { display: block; font: 500 16px var(--f-mono); font-variant-numeric: tabular-nums; }
.steer-read span { font-size: 11px; color: var(--muted); }
.parts { list-style: none; margin: 0; padding: 0; font-size: 12.5px; }
.parts li { display: flex; justify-content: space-between; gap: 8px; padding: 4px 6px; border-radius: 3px; cursor: pointer; }
.parts li:hover, .parts li.on { background: color-mix(in srgb, var(--accent) 16%, transparent); }
.parts li span:last-child { font-family: var(--f-mono); color: var(--muted); font-variant-numeric: tabular-nums; }
.parts .grp { font: 600 11px var(--f-display); letter-spacing: 0.1em; text-transform: uppercase; color: var(--muted); cursor: default; padding-top: 10px; }
.parts .grp:hover { background: none; }
.sw { width: 9px; height: 9px; border-radius: 2px; display: inline-block; margin-right: 7px; vertical-align: 0; }
.note { font-size: 12.5px; color: var(--muted); margin: 0; }
a.btn { text-decoration: none; display: inline-block; }
button, a.btn { font: 600 12px var(--f-body); color: var(--ink); background: transparent; border: 1px solid var(--line); border-radius: 4px; padding: 5px 10px; cursor: pointer; }
button:hover { border-color: var(--accent); }
:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
@media (max-width: 820px) {
  .app { flex-direction: column; height: auto; }
  .stage { height: 62vh; flex: none; }
  aside { width: auto; border-left: 0; border-top: 1px solid var(--line); }
}
</style>

<div class="app">
  <div class="stage">
    <canvas id="cv" aria-label="3D model of the robot. Drag to orbit, scroll to zoom."></canvas>
    <div class="hud">
      <h1>Autonav R1</h1>
      <p>Designed by theretroexe for the Autonomous Navigation Challenge. Drag to orbit, right-drag to pan, scroll to zoom, hover a part to name it.</p>
    </div>
    <div class="legend" id="legend"></div>
    <div class="tip" id="tip" hidden></div>
  </div>
  <aside>
    <section>
      <h2>Start here</h2>
      <p class="note">Everything you need to build the robot is on this page. Download the files, print the plates in order, then follow the assembly steps below.</p>
      <div style="display:flex; gap:8px; margin:10px 0; align-items:center; flex-wrap:wrap">
        <a class="btn" id="dl" href="handoff/autonav_robot_files.zip" download>Download build files (.zip, __ZIPMB__ MB)</a>
        <span class="note" id="dlMsg" role="status"></span>
      </div>
      <p class="note">The zip has a Bambu Studio project with the 8 plates already laid out (<code>autonav_plates_BambuStudio.3mf</code>), every part as an STL for other slicers, print notes (<code>PRINT_PLATES.md</code>), and a 3D model plus dimension sheet (<code>autonav_robot.obj</code>, <code>autonav_robot.md</code>) in mm.</p>
      <ol class="note" id="plates" start="0" style="padding-left:18px; margin:10px 0"></ol>
      <p class="note">Print plate 0 first and push its pins into its holes. If they're too tight or too loose, sort that out before printing the rest: every pin and joint uses the same clearance. Use 0.12 mm layers for the gears (plate 5). Have ready: superglue, double-sided tape, hot glue, a rubber band and a little grease.</p>
    </section>
    <section>
      <h2>Assembly, step by step</h2>
      <div class="ctl">
        <label for="step" id="stepTitle">Complete robot</label><output id="stepOut"></output>
        <input type="range" id="step" min="1" max="1" step="1" value="1">
      </div>
      <p class="note" id="stepText">Drag the slider or use the buttons to build the robot one step at a time. New parts drop in highlighted.</p>
      <div style="display:flex; gap:8px; margin-top:10px">
        <button id="stepPrev">◀ Back</button><button id="stepNext">Next ▶</button><button id="stepAll">Show complete</button>
      </div>
    </section>
    <section>
      <h2>Rulebook check</h2>
      <table id="rules"></table>
    </section>
    <section>
      <h2>Steering</h2>
      <div class="ctl">
        <label for="steer">Servo horn angle</label><output id="steerOut">0°</output>
        <input type="range" id="steer" min="-32" max="32" step="1" value="0">
      </div>
      <div class="steer-read">
        <div><b id="sIn">0.0°</b><span>inner wheel</span></div>
        <div><b id="sOut">0.0°</b><span>outer wheel</span></div>
        <div><b id="sErr">0.0°</b><span>off true Ackermann</span></div>
      </div>
      <div class="checks"><label><input type="checkbox" id="anim"> Sweep lock to lock</label></div>
    </section>
    <section>
      <h2>View</h2>
      <div class="ctl">
        <label for="explode">Separate the layers</label><output id="explodeOut">0 mm</output>
        <input type="range" id="explode" min="0" max="90" step="1" value="0">
      </div>
      <div class="checks">
        <label><input type="checkbox" id="bought" checked> Show bought parts (electronics, lidar, servos, motor)</label>
        <label><input type="checkbox" id="env"> Show 300 mm size box, 100 mm wall height, lidar scan plane</label>
        <label><input type="checkbox" id="xray"> See through printed parts</label>
      </div>
      <div style="display:flex; gap:8px; margin-top:10px; flex-wrap:wrap">
        <button id="vIso">3/4 front</button><button id="vSide">Side</button><button id="vTop">Top</button><button id="vUnder">Underside</button>
      </div>
    </section>
    <section>
      <h2>Fasteners</h2>
      <table id="fast"></table>
    </section>
    <section>
      <h2>Parts</h2>
      <ul class="parts" id="parts"></ul>
    </section>
    <section>
      <h2>Still to confirm</h2>
      <p class="note">Motor RPM is unknown, so the 36:11 drive ratio is a reprintable pinion and crown. Print a test of each servo horn first: their splines are printed (25-tooth MG996R, 21-tooth SG90) and the fit is one setting in robot.py. Unmeasured: MG996R flange height and the Brio clip. Print snap pins, standoffs, horns and differential gears in PETG; tyres in TPU; the rest in PLA. Printed masses are estimates, so weigh the real parts.</p>
    </section>
  </aside>
</div>

<script type="importmap">
{"imports": {"three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
             "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}
</script>
<script id="data" type="application/json">__DATA__</script>
<script id="glb" type="text/plain">__GLB__</script>
<script type="module">
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

const D = JSON.parse(document.getElementById("data").textContent);
const G = D.geo, REP = D.rep;
const GROUP_IDX = { chassis: 0, mid: 1, cam: 1, top: 2 };
const LABEL = n => n.replace(/_/g, " ");
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim();

// ---- rule table
const env = REP.envelope_mm;
const rows = [
  ["Size limit 300 × 300 × 300 mm (at full steering lock)", `${env.L} × ${env.W} × ${env.H}`, "pass"],
  ["Mass limit 1.5 kg", `${(REP.total_mass_g / 1000).toFixed(2)} kg`, "est"],
  ["Lidar scan plane under the 100 mm walls and obstacles", `${REP.lidar_scan_plane_z.toFixed(1)} mm`, "pass"],
  ["Emergency stop and start button (required): the team hot-glues them on, not modelled", "to add", ""],
  ["IR line sensors under the plate, front + rear (face ~9 mm off the floor)", "2", ""],
  ["Steering lock, inner / outer wheel", `${REP.lock_deg_inner_outer[0]}° / ${REP.lock_deg_inner_outer[1]}°`, ""],
  ["Tightest turn radius (track is 1000 mm wide)", `${REP.min_turn_radius_mm} mm`, ""],
  ["Drive: crown : pinion, open differential", REP.drive_ratio.split(" =")[0], ""],
  ["Scrub radius (kingpin sits inside the wheel)", `${REP.scrub_radius_mm} mm`, ""],
  ["Printed filament, PLA estimate (v1 was " + REP.printed_mass_v1_g + " g)", `${REP.printed_mass_g} g`, "est"],
  [`Every printed part fits the P2S bed (≤ ${REP.bed.limit_mm} mm); decks split with glued dovetails`,
   `largest ${Object.values(REP.bed.largest)[0].slice(1).join(" × ")}`, "pass"],
  [`Moving parts clear their neighbours by ≥ ${REP.running_clearance.min_gap_mm} mm (pin/bore fit ${REP.running_clearance.FIT} mm)`,
   `${REP.running_clearance.tight.length} tight`, "pass"],
  ["Gear teeth never overlap through a full tooth pitch (crown/pinion, bevels)",
   `${Math.max(...Object.values(REP.gear_mesh_overlap_mm3)).toFixed(3)} mm³`, "pass"],
  [`Drivetrain strength: weakest part (${REP.strength.weakest}) at 2× the tyre-grip limit`,
   `${REP.strength.min_SF}× margin`, "pass"],
  ["Diff goes together: every piece slides into place without hitting anything",
   `${Math.max(...Object.values(REP.diff_assembly_overlap_mm3)).toFixed(3)} mm³`, "pass"],
  ["Diff gears can't slide out of mesh (max axial float)", `${Math.max(...Object.values(REP.gear_float_mm)).toFixed(2)} mm`, "pass"],
  ["Collisions at rest and across the steering sweep (> 0.01 mm³)",
   `${REP.interference_static.length + REP.interference_steering.length}`, "pass"],
];
const F = REP.fasteners;
document.getElementById("fast").innerHTML =
  `<tr><td><b>Screws, nuts or inserts to buy</b></td><td class="v">none</td></tr>` +
  `<tr><td><b>Printed 4 mm snap pins</b> (lengths ${F.pin_lengths_mm.join(", ")} mm)</td><td class="v">${F.snap_pins}</td></tr>` +
  Object.entries(F.joints).map(([k, v]) => `<tr><td>&nbsp;&nbsp;${k}</td><td class="v">${v}</td></tr>`).join("") +
  `<tr><td colspan="2">Also printed: ${F.printed_fasteners.join(", ")}.</td></tr>` +
  `<tr><td colspan="2">Hot glue, superglue or double-sided tape: ${F.glue_or_tape.join(", ")}.</td></tr>`;
document.getElementById("rules").innerHTML = rows.map(([k, v, s]) =>
  `<tr><td>${k}${s ? `<span class="pill ${s}">${s === "pass" ? "PASS" : "ESTIMATE"}</span>` : ""}</td><td class="v">${v}</td></tr>`).join("");

// ---- scene
const cv = document.getElementById("cv");
const renderer = new THREE.WebGLRenderer({ canvas: cv, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(32, 1, 5, 5000);
const controls = new OrbitControls(camera, cv);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x5a5f66, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(300, 600, 200); scene.add(sun);
const fill = new THREE.DirectionalLight(0xffffff, 0.6); fill.position.set(-400, 200, -300); scene.add(fill);
const root = new THREE.Group(); root.rotation.x = -Math.PI / 2; scene.add(root);   // model is Z-up, mm

const grid = new THREE.GridHelper(600, 30); grid.position.set(0, 0, -100); scene.add(grid);

// envelope helpers (model coords)
const envGroup = new THREE.Group(); envGroup.visible = false; root.add(envGroup);
{
  const box = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(300, 300, 300)),
    new THREE.LineBasicMaterial({ color: 0xd9661a }));
  box.position.set(0, 101, 150); envGroup.add(box);
  const wall = new THREE.Mesh(new THREE.PlaneGeometry(300, 300), new THREE.MeshBasicMaterial({ color: 0x222222, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false }));
  wall.position.set(0, 101, 100); envGroup.add(wall);
  const scan = new THREE.Mesh(new THREE.CircleGeometry(150, 96), new THREE.MeshBasicMaterial({ color: 0xff3b30, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false }));
  scan.position.set(0, G.LIDAR_Y, G.SCAN); envGroup.add(scan);
}

const meshes = {};       // name -> { obj, mat, info, base matrix }
const glbBytes = Uint8Array.from(atob(document.getElementById("glb").textContent.trim()), c => c.charCodeAt(0));
new GLTFLoader().parse(glbBytes.buffer, "", gltf => {
  gltf.scene.traverse(o => {
    if (!o.isMesh) return;
    const name = o.name.split("__").pop();
    const info = D.parts[name]; if (!info) return;
    const mat = new THREE.MeshStandardMaterial({ color: info.color, roughness: info.printed ? 0.7 : 0.45,
      metalness: name.includes("motor") && !info.printed ? 0.6 : 0.05, flatShading: true });
    const mesh = new THREE.Mesh(o.geometry, mat);
    mesh.name = name; mesh.matrixAutoUpdate = false;
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(o.geometry, 28),
      new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.22 }));
    mesh.add(edges);
    root.add(mesh);
    meshes[name] = { obj: mesh, mat, info };
  });
  buildPartList(); setStep(NSTEP + 1); setView("iso");
}, err => { document.querySelector(".hud p").textContent = "The 3D model failed to load: " + err; });

// ---- kinematics (same maths as robot.py)
const rot = (v, deg) => { const a = deg * Math.PI / 180, c = Math.cos(a), s = Math.sin(a); return [v[0] * c - v[1] * s, v[0] * s + v[1] * c]; };
const add2 = (a, b) => [a[0] + b[0], a[1] + b[1]], sub2 = (a, b) => [a[0] - b[0], a[1] - b[1]];
function steerAt(phi) {
  const i = Math.round(phi) + (D.steer.length - 1) / 2;
  const [, l, r] = D.steer[Math.max(0, Math.min(D.steer.length - 1, i))];
  return [l, r];
}
const M4 = () => new THREE.Matrix4();
const pivotZ = (x, y, deg) => M4().makeTranslation(x, y, 0).multiply(M4().makeRotationZ(deg * Math.PI / 180)).multiply(M4().makeTranslation(-x, -y, 0));

let phi = 0, explode = 0, xray = false, showBought = true, selected = null;
const NSTEP = D.steps.length;
let step = NSTEP + 1, stepT0 = 0, dropP = 1;          // step NSTEP+1 = complete robot
function update() {
  const [dl, dr] = steerAt(phi);
  const S = G.S;
  const drop = 60 * (1 - dropP) ** 3, cur = D.steps[step - 1], focus = cur && cur[2], dir = cur ? cur[3] : [0, 0, 1];
  for (const [name, m] of Object.entries(meshes)) {
    let T = M4();
    const side = name.endsWith("_L") ? -1 : name.endsWith("_R") ? 1 : 0;
    const d = side < 0 ? dl : dr;
    if (/^(knuckle|wheel_front|tire_front|axle_clip_front|pivot_pin_arm)/.test(name)) T = pivotZ(side * G.KP_X, G.WB, d);
    else if (name === "horn_adapter" || name.startsWith("pivot_pin_horn")) T = pivotZ(S[0], S[1], phi);
    else if (name.startsWith("tie_rod")) {
      const K = [side * G.KP_X, G.WB], A0 = [side * G.ARM[0], G.ARM[1]], H0 = [side * G.PIV, G.ARM[1]];
      const A = add2(K, rot(sub2(A0, K), d)), H = add2(S, rot(sub2(H0, S), phi));
      const th = Math.atan2(A[1] - H[1], A[0] - H[0]) - Math.atan2(A0[1] - H0[1], A0[0] - H0[0]);
      T = M4().makeTranslation(H[0], H[1], 0).multiply(M4().makeRotationZ(th)).multiply(M4().makeTranslation(-H0[0], -H0[1], 0));
    }
    const isNew = m.info.step === step;
    const dd = isNew ? drop : 0;
    const lift = M4().makeTranslation(dir[0] * dd, dir[1] * dd, explode * GROUP_IDX[m.info.group] + dir[2] * dd);
    m.obj.matrix.copy(lift.multiply(T));
    m.obj.matrixWorldNeedsUpdate = true;
    m.obj.visible = (m.info.printed || showBought) && m.info.step <= step && (!focus || D.steps[m.info.step - 1][2]);
    m.mat.transparent = xray && m.info.printed; m.mat.opacity = m.mat.transparent ? 0.28 : 1; m.mat.depthWrite = !m.mat.transparent;
    m.mat.emissive.set(name === selected ? 0x553311 : isNew ? 0x1e4a8c : 0x000000);
  }
  const inner = Math.max(Math.abs(dl), Math.abs(dr)), outer = Math.min(Math.abs(dl), Math.abs(dr));
  const ideal = inner > 0.01 ? Math.atan(1 / (1 / Math.tan(inner * Math.PI / 180) + 2 * G.KP_X / G.WB)) * 180 / Math.PI : 0;
  document.getElementById("sIn").textContent = inner.toFixed(1) + "°";
  document.getElementById("sOut").textContent = outer.toFixed(1) + "°";
  document.getElementById("sErr").textContent = (outer - ideal).toFixed(2) + "°";
  document.getElementById("steerOut").textContent = `${phi > 0 ? "+" : ""}${phi}°`;
  document.getElementById("explodeOut").textContent = `${explode} mm`;
}

// ---- part list + legend
function buildPartList() {
  const groups = [["chassis", "Chassis layer"], ["mid", "Electronics + camera deck"], ["top", "Rear top deck"], ["cam", "Camera"]];
  const ul = document.getElementById("parts");
  let html = "";
  for (const [g, title] of groups) {
    for (const printed of [true, false]) {
      const list = Object.entries(D.parts).filter(([, p]) => p.group === g && p.printed === printed);
      if (!list.length) continue;
      const sum = list.reduce((s, [, p]) => s + (p.g || 0), 0);
      html += `<li class="grp"><span>${title}, ${printed ? "printed" : "bought"}</span><span>${Math.round(sum)} g</span></li>`;
      html += list.map(([n, p]) => `<li data-n="${n}" tabindex="0"><span><i class="sw" style="background:${p.color}"></i>${LABEL(n)}</span><span>${p.g ?? "-"} g</span></li>`).join("");
    }
  }
  ul.innerHTML = html;
  ul.querySelectorAll("li[data-n]").forEach(li => {
    const pick = () => { select(li.dataset.n === selected ? null : li.dataset.n); };
    li.addEventListener("click", pick);
    li.addEventListener("keydown", e => { if (e.key === "Enter" || e.key === " ") { e.preventDefault(); pick(); } });
  });
  const leg = [["#f5821e", "gears"], ["#2878dc", "brackets"], ["#5aa0f0", "steering links"], ["#fac83c", "printed pins and clips"], ["#ebebeb", "standoffs"], ["#3c4048", "decks"], ["#146e3c", "electronics"]];
  document.getElementById("legend").innerHTML = leg.map(([c, t]) => `<span><i style="background:${c}"></i>${t}</span>`).join("");
}
function select(n) {
  selected = n;
  document.querySelectorAll(".parts li[data-n]").forEach(li => li.classList.toggle("on", li.dataset.n === n));
  update();
}

// ---- interaction
const tip = document.getElementById("tip"), ray = new THREE.Raycaster(), ptr = new THREE.Vector2();
function hit(e) {
  const r = cv.getBoundingClientRect();
  ptr.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  ray.setFromCamera(ptr, camera);
  const objs = Object.values(meshes).filter(m => m.obj.visible).map(m => m.obj);
  const h = ray.intersectObjects(objs, false)[0];
  return [h ? h.object.name : null, e.clientX - r.left, e.clientY - r.top];
}
cv.addEventListener("pointermove", e => {
  const [n, x, y] = hit(e);
  tip.hidden = !n; if (n) { tip.textContent = LABEL(n); tip.style.left = x + "px"; tip.style.top = y + "px"; }
});
cv.addEventListener("pointerleave", () => { tip.hidden = true; });
let down = null;
cv.addEventListener("pointerdown", e => { down = [e.clientX, e.clientY]; });
cv.addEventListener("pointerup", e => {
  if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4) return;
  const [n] = hit(e); select(n === selected ? null : n);
  const li = n && document.querySelector(`.parts li[data-n="${n}"]`); if (li) li.scrollIntoView({ block: "nearest" });
});
const $ = id => document.getElementById(id);
$("steer").addEventListener("input", e => { phi = +e.target.value; update(); });
$("explode").addEventListener("input", e => { explode = +e.target.value; update(); });
$("bought").addEventListener("change", e => { showBought = e.target.checked; update(); });
$("xray").addEventListener("change", e => { xray = e.target.checked; update(); });
$("env").addEventListener("change", e => { envGroup.visible = e.target.checked; });
const reduce = matchMedia("(prefers-reduced-motion: reduce)").matches;
let lastFocus = false;
function setStep(s) {
  step = Math.max(1, Math.min(NSTEP + 1, s)); $("step").value = step;
  const done = step > NSTEP, st = D.steps[step - 1];
  $("stepTitle").textContent = done ? "Complete robot" : st[0];
  const focus = !done && st[2];
  if (focus !== lastFocus) { setView(focus ? "diff" : "iso"); lastFocus = focus; }
  $("stepOut").textContent = done ? `${NSTEP} steps` : `${step} / ${NSTEP}`;
  $("stepText").textContent = done ? "Every part fitted. Step back to see how it goes together." : st[1];
  $("stepPrev").disabled = step === 1; $("stepNext").disabled = done;
  stepT0 = performance.now(); dropP = done || reduce ? 1 : 0; update();
}
$("step").max = NSTEP + 1;
$("step").addEventListener("input", e => setStep(+e.target.value));
$("stepPrev").onclick = () => setStep(step - 1);
$("stepNext").onclick = () => setStep(step + 1);
$("stepAll").onclick = () => setStep(NSTEP + 1);
let t0 = performance.now();

// views: model coords (x right, y forward, z up) -> three (x, z, -y)
const TARGET = new THREE.Vector3(0, 95, -105), DIFF_T = new THREE.Vector3(0, G.AX_Z, 0);
const VIEWS = { iso: [430, 300, -520], side: [620, 110, -105], top: [0, 760, -104], under: [260, -420, -330], diff: [-125, 85, 75] };
function setView(k) { camera.position.set(...VIEWS[k]); controls.target.copy(k === "diff" ? DIFF_T : TARGET); controls.update(); }
$("vIso").onclick = () => setView("iso"); $("vSide").onclick = () => setView("side");
$("vTop").onclick = () => setView("top"); $("vUnder").onclick = () => setView("under");

function resize() {
  const r = cv.parentElement.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false); camera.aspect = r.width / r.height; camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(cv.parentElement);
function paintBg() {
  scene.background = new THREE.Color(css("--scene") || "#e3e7eb");
  const dark = new THREE.Color(css("--scene")).getHSL({}).l < 0.5;
  grid.material.color = new THREE.Color(dark ? 0x30353c : 0xc4cad1); grid.material.vertexColors = false;
}
paintBg(); matchMedia("(prefers-color-scheme: dark)").addEventListener("change", paintBg);
new MutationObserver(paintBg).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });

renderer.setAnimationLoop(now => {
  if ($("anim").checked && !reduce) {
    phi = Math.round(32 * Math.sin((now - t0) / 900)); $("steer").value = phi; update();
  }
  if (dropP < 1) { dropP = Math.min(1, (now - stepT0) / 700); update(); }
  controls.update(); renderer.render(scene, camera);
});
</script>
<script type="module">
// plate list + zip download (inside claude.ai the page must use the downloads capability; locally the plain link works)
const D = JSON.parse(document.getElementById("data").textContent);
document.getElementById("plates").innerHTML = D.plates.map(([n, f]) => `<li><b>${f}</b>: ${n.replace(/^\d+ \S+ /, "").replace(" - ", ", ")}</li>`).join("");
const dl = document.getElementById("dl"), msg = document.getElementById("dlMsg");
const downloads = window.claude?.use ? await window.claude.use("downloads") : null;
if (downloads) dl.addEventListener("click", async (e) => {
  e.preventDefault();
  msg.textContent = "Preparing…";
  try {
    const r = await fetch(dl.getAttribute("href") + ".b64.txt");
    if (!r.ok) throw new Error(r.status);
    const blob = await (await fetch("data:application/zip;base64," + (await r.text()).trim())).blob();
    await downloads.save({ filename: "autonav_robot_files.zip", data: blob });
    msg.textContent = "Saved.";
  } catch (err) {
    msg.textContent = err?.code === "declined" ? "" : "Download didn't work here. Ask the owner for the zip.";
  }
});
</script>
"""

OUT = R.OUT
glb = base64.b64encode(open(os.path.join(OUT, "robot.glb"), "rb").read()).decode()
rep = json.load(open(os.path.join(OUT, "report.json")))
P = R.build()
grams = rep["parts_printed_g"]
DIFF = (True, [-1, 0, 0])    # close-up of the diff on its own; parts slide in along the axle from the cover side
STEPS = [  # (title, what to do, part-name regexes[, (focus, drop dir)]); every part must land in exactly one step
    ("Join the bottom plate", "Superglue the two bottom plate halves at the dovetail. Keep them pressed flat on a table while the glue sets.",
     [r"bottom_plate_"]),
    ("Diff 1/4: crown half", "Build the differential on the table first. Lay the crown half with its gear teeth down, so the open split face with its 3 pegs and 3 grooves points up (toward you in this view).",
     [r"diff_case_crown"], DIFF),
    ("Diff 2/4: first side gear", "Drop a side gear in hub first: the hub goes into the bore, the teeth face up into the cavity. Both side gears are the same part. The hex hole in the hub takes the half shaft later.",
     [r"side_gear_R"], DIFF),
    ("Diff 3/4: spider cross", "Slide a spider gear onto each arm of the 3-arm cross, teeth toward the centre (all 3 spiders are the same part). Put the cross in flat face up, so the arms drop into the 3 grooves in the split face. Turn the side gear a little until all three spiders mesh and the cross sits flat.",
     [r"spider_"], DIFF),
    ("Diff 4/4: cover", "Drop the other side gear into the cover the same way, hub first. Turn the cover over and lower it onto the crown half so the 3 pegs enter its holes. If the teeth don't line up, turn the side gear a touch. Don't glue it: tape or a rubber band holds it until the rear housing clamps it in step 6. Test: hold the case and turn one hex hole with a 7 mm hex or the shaft. The other side gear should turn the opposite way, smoothly. A little grease on the teeth helps.",
     [r"side_gear_L", r"diff_cover"], DIFF),
    ("Rear axle", "Push each half shaft's hex end through the case collar into a side gear. Lay the differential and shafts into the rear housing, crown teeth toward the motor side (right), with the collars in the inner uprights. Put the axle caps on top and push 8 snap pins through the caps, the housing and the plate.",
     [r"rear_housing", r"half_shaft_", r"axle_cap_", r"snap_pin_0[0-7]$"]),
    ("Drive motor", "Press the pinion onto the motor's D-shaft and snap the motor into the mount's ring. Pin the mount to the plate with 4 snap pins, then check that the pinion meshes with the crown.",
     [r"motor_", r"JGA25", r"snap_pin_(08|09|10|11)$"]),
    ("Front axle and steering", "Pin the front axle to the plate (4 snap pins). Fit each knuckle with its kingpin. Snap the MG996R into its hooks, press the printed horn onto the spline, then join the tie rods to the horn and the knuckle arms with the pivot pins.",
     [r"front_axle", r"kingpin_", r"knuckle_", r"MG996R", r"horn_adapter", r"tie_rod_", r"pivot_pin_", r"snap_pin_1[6-9]$"]),
    ("Wheels and tyres", "Stretch the TPU tyres onto the wheels, slide each wheel onto its shaft and push the axle clip on.",
     [r"wheel_", r"tire_", r"axle_clip_"]),
    ("Lidar, battery, line sensors", "Pin the lidar cradle to the front of the plate (4 snap pins). Tape or hot-glue the RPLIDAR C1, the LiPo and both IR line sensors in place.",
     [r"lidar_cradle", r"RPLIDAR", r"LiPo", r"IR_", r"snap_pin_1[2-5]$"]),
    ("Lower standoffs", "Snap the 6 lower standoffs into the bottom plate.",
     [r"standoff_low_"]),
    ("Mid deck and electronics", "Superglue the mid deck dovetail and press the deck onto the standoff pegs. Double-sided tape the Pi, ESP32, IBT-2, buck converters, IMU and lidar USB adapter.",
     [r"mid_deck_", r"Raspberry", r"ESP32", r"IBT2", r"buck_", r"lidar_usb", r"IMU"]),
    ("Camera", "Hook the SG90 into the deck, put the cradle on its horn and clip the Brio 100 into the cradle.",
     [r"SG90", r"cam_cradle", r"Logitech"]),
    ("Rear top deck", "Snap the 4 tall standoffs into the mid deck and press the top deck onto them.",
     [r"standoff_high_", r"top_deck"]),
]
step_of = {}
for n in P:
    hits = [i + 1 for i, st in enumerate(STEPS) if any(re.match(p, n) for p in st[2])]
    assert len(hits) == 1, f"{n} is in steps {hits}"
    step_of[n] = hits[0]
parts = {n: dict(group=g, printed=pr, color="#%02x%02x%02x" % c, g=grams.get(n, R.BOUGHT_MASS.get(n)), step=step_of[n])
         for n, (m, c, pr, g) in P.items()}
steer = [[phi, *(round(v, 3) for v in (R.steer_solution(phi) if phi else (0.0, 0.0)))]
         for phi in range(-int(R.HORN_MAX), int(R.HORN_MAX) + 1)]
geo = dict(KP_X=R.KP_X, WB=R.WB, S=[float(v) for v in R.SERVO_SHAFT], ARM=[float(v) for v in R.ARM_END],
           PIV=R.HORN_PIV, SCAN=R.LIDAR_SCAN, LIDAR_Y=R.LIDAR_Y, AX_Z=R.AX_Z)
data = dict(parts=parts, steps=[[*s[:2], *(s[3] if len(s) > 3 else (False, [0, 0, 1]))] for s in STEPS], steer=steer, geo=geo, rep={k: rep[k] for k in (
    "envelope_mm", "printed_mass_g", "bought_mass_g", "total_mass_g", "lock_deg_inner_outer",
    "min_turn_radius_mm", "drive_ratio", "lidar_scan_plane_z", "ground_clearance_min",
    "fasteners", "printed_mass_v1_g", "scrub_radius_mm", "bed", "running_clearance", "gear_mesh_overlap_mm3",
    "interference_static", "interference_steering", "strength", "gear_float_mm", "diff_assembly_overlap_mm3")})
data["plates"] = [p[:2] for p in make_plates.PLATES]
zmb = os.path.getsize(os.path.join(OUT, "handoff", "autonav_robot_files.zip")) / 2**20   # run make_handoff.py first
html = (PAGE.replace("__DATA__", json.dumps(data).replace("</", "<\\/")).replace("__GLB__", glb)
        .replace("__ZIPMB__", f"{zmb:.1f}"))
open(os.path.join(OUT, "robot_viewer.html"), "w", encoding="utf-8").write(html)
print("ok", round(len(html) / 1e6, 2), "MB")
