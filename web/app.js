import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';

/* ============================================================================
   helpers
   ========================================================================== */
const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];
const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const fmtT = (t) => { const s = Math.max(0, t || 0); const m = Math.floor(s / 60); return `${m}:${(s - m * 60).toFixed(1).padStart(4, '0')}`; };
const fmtD = (t) => { const s = Math.round(t || 0); return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`; };

async function api(path, method = 'GET', body) {
  const opt = { method, headers: {} };
  if (body !== undefined) { opt.body = JSON.stringify(body); opt.headers['Content-Type'] = 'application/json'; }
  const r = await fetch(path, opt);
  let data = null;
  try { data = await r.json(); } catch { /* empty */ }
  if (!r.ok) { const e = new Error((data && data.error) || `${r.status} ${r.statusText}`); e.confirm = !!(data && data.confirm); throw e; }
  return data;
}
let toastTimer = null;
function toast(msg, err = false) {
  const el = $('#toast');
  el.textContent = msg; el.className = 'show' + (err ? ' err' : '');
  clearTimeout(toastTimer); toastTimer = setTimeout(() => (el.className = ''), err ? 6000 : 2500);
}
async function act(fn, okMsg) {
  try { const r = await fn(); if (okMsg) toast(okMsg); return r; } catch (e) { toast(e.message, true); return null; }
}

const STATUS_RGB = ['#39414f', '#6c7a96', '#2ecc71', '#f1c40f', '#3fa9f5', '#ff9f1a', '#ff5e3a', '#9b8cff', '#2a9d8f', '#ff2d55'];
const STATUS_LABEL = ['offline', 'connected', 'ready', 'armed', 'flying', 'warning', 'failsafe', 'landed', 'spare', 'error'];
const STATUS_F = STATUS_RGB.map((h) => new THREE.Color(h));
const KIND_COLORS = { takeoff: '#3b5b7a', hold: '#2a8f6a', transition: '#40507a', spin: '#7a4fa8', landing: '#3b5b7a', path: '#4a6aa0' };

/* ============================================================================
   app state
   ========================================================================== */
let info = null, spec = null, design = null, status = null;
let selected = -1, tab = 'design', dirty = false, saveTimer = null;
let playing = false, playT = 0, speed = 1, inflight = false, wantT = 0, lastFrameKind = 0;
let selectedDrone = -1, layoutData = null, framed = false;

/* ============================================================================
   3D scene
   ========================================================================== */
const view = $('#viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
view.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color('#02040a');
scene.fog = new THREE.FogExp2('#02040a', 0.0011);
const camera = new THREE.PerspectiveCamera(50, 1, 0.5, 6000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI * 0.495;
const world = new THREE.Group();
scene.add(world);

function glowTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 64;
  const g = c.getContext('2d');
  const grd = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  grd.addColorStop(0, 'rgba(255,255,255,1)'); grd.addColorStop(0.18, 'rgba(255,255,255,0.9)');
  grd.addColorStop(0.45, 'rgba(255,255,255,0.25)'); grd.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grd; g.fillRect(0, 0, 64, 64);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
}
let cap = 0, nDrones = 0;
const dGeom = new THREE.BufferGeometry();
const dMat = new THREE.PointsMaterial({ size: 3.2, map: glowTexture(), vertexColors: true, transparent: true,
  depthWrite: false, blending: THREE.AdditiveBlending, sizeAttenuation: true });
const dPoints = new THREE.Points(dGeom, dMat);
dPoints.frustumCulled = false;
scene.add(dPoints);
let posArr = new Float32Array(0), colArr = new Float32Array(0), lastStatus = new Uint8Array(0);
function ensureCap(n) {
  if (n <= cap) return;
  cap = Math.max(n, Math.ceil(cap * 1.5), 64);
  posArr = new Float32Array(cap * 3); colArr = new Float32Array(cap * 3);
  dGeom.setAttribute('position', new THREE.BufferAttribute(posArr, 3).setUsage(THREE.DynamicDrawUsage));
  dGeom.setAttribute('color', new THREE.BufferAttribute(colArr, 3).setUsage(THREE.DynamicDrawUsage));
}
ensureCap(64);
const selMarker = new THREE.Mesh(new THREE.RingGeometry(2.2, 2.8, 32), new THREE.MeshBasicMaterial({ color: '#4cc3ff', side: THREE.DoubleSide, transparent: true, opacity: 0.9, depthTest: false }));
selMarker.visible = false; scene.add(selMarker);

function clearGroup(g) { while (g.children.length) { const o = g.children.pop(); o.geometry?.dispose?.(); o.material?.dispose?.(); } }
function S2T(x, y, z) { return new THREE.Vector3(x, z, -y); }

function buildField(settings, pads) {
  clearGroup(world);
  const g = settings.geofence;
  const gx = g.x, gy = g.y, gz = g.z;
  // ground
  const size = Math.max(gx[1] - gx[0], gy[1] - gy[0], 600) * 2.2;
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(size, size), new THREE.MeshBasicMaterial({ color: '#05080f' }));
  ground.rotation.x = -Math.PI / 2; ground.position.y = -0.05; world.add(ground);
  const grid = new THREE.GridHelper(size, Math.round(size / 10), '#16203a', '#0c1324');
  world.add(grid);
  // geofence box
  const box = new THREE.Box3(S2T(gx[0], gy[1], gz[0]), S2T(gx[1], gy[0], gz[1]));
  const bh = new THREE.Box3Helper(box, new THREE.Color('#1f6a9a')); bh.material.transparent = true; bh.material.opacity = 0.45;
  world.add(bh);
  // audience zone + safety line
  const ay = settings.audience_y, ab = settings.audience_buffer;
  const az = new THREE.Mesh(new THREE.PlaneGeometry(gx[1] - gx[0] + 120, 80),
    new THREE.MeshBasicMaterial({ color: '#ff2d55', transparent: true, opacity: 0.16, depthWrite: false }));
  az.rotation.x = -Math.PI / 2; az.position.copy(S2T((gx[0] + gx[1]) / 2, ay - 40, 0.02)); world.add(az);
  const lineGeo = new THREE.BufferGeometry().setFromPoints([S2T(gx[0] - 60, ay + ab, 0.05), S2T(gx[1] + 60, ay + ab, 0.05)]);
  world.add(new THREE.Line(lineGeo, new THREE.LineDashedMaterial({ color: '#ff5e7a', dashSize: 4, gapSize: 3 })).computeLineDistances());
  const al = new THREE.BufferGeometry().setFromPoints([S2T(gx[0] - 60, ay, 0.05), S2T(gx[1] + 60, ay, 0.05)]);
  world.add(new THREE.Line(al, new THREE.LineBasicMaterial({ color: '#ff2d55' })));
  // pads
  if (pads && pads.pads) {
    const arr = new Float32Array(pads.pads.length * 3);
    const col = new Float32Array(pads.pads.length * 3);
    pads.pads.forEach((p, i) => {
      arr[i * 3] = p[0]; arr[i * 3 + 1] = 0.03; arr[i * 3 + 2] = -p[1];
      const c = pads.spare[i] ? [0.1, 0.45, 0.4] : [0.22, 0.26, 0.36];
      col.set(c, i * 3);
    });
    const pg = new THREE.BufferGeometry();
    pg.setAttribute('position', new THREE.BufferAttribute(arr, 3));
    pg.setAttribute('color', new THREE.BufferAttribute(col, 3));
    world.add(new THREE.Points(pg, new THREE.PointsMaterial({ size: 1.1, vertexColors: true, sizeAttenuation: true })));
  }
}

function setCam(kind) {
  const s = (layoutData && layoutData.settings) || (info && info.defaults);
  const ay = s ? s.audience_y : -150;
  const ext = design && design.validation ? design.validation.stats.extent : { x: [-120, 120], y: [-60, 80], z: [0, 140] };
  const w = Math.max(ext.x[1] - ext.x[0], 120), hgt = Math.max(ext.z[1], 80);
  const vfov = THREE.MathUtils.degToRad(camera.fov);
  const hfov = 2 * Math.atan(Math.tan(vfov / 2) * camera.aspect);
  const dist = Math.max(-ay + 60, (w * 0.62) / Math.tan(hfov / 2), (hgt * 0.62) / Math.tan(vfov / 2));
  const cz = hgt * 0.5;
  if (kind === 'top') { camera.position.set(0, Math.max(w, 300) * 1.25, 1); controls.target.set(0, 0, 0); }
  else if (kind === 'side') { camera.position.set(dist, cz * 0.8, -20); controls.target.set(0, cz, -20); }
  else { camera.position.set(0, 4, dist); controls.target.set(0, cz, 0); }
  controls.update();
}

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h, false); renderer.domElement.style.width = w + 'px'; renderer.domElement.style.height = h + 'px';
  camera.aspect = w / Math.max(1, h); camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);

function applyFrame(kind, n, t, pos, col, st) {
  ensureCap(n);
  nDrones = n; lastFrameKind = kind;
  const showDark = $('#showDark').checked;
  const statusMode = kind === 2 && $('#statusColors').checked;
  if (lastStatus.length < n) lastStatus = new Uint8Array(cap);
  for (let i = 0; i < n; i++) {
    const s = st[i];
    lastStatus[i] = s;
    const j = i * 3;
    if (kind === 2 && s === 0) { posArr[j] = 0; posArr[j + 1] = -5000; posArr[j + 2] = 0; continue; }
    posArr[j] = pos[j]; posArr[j + 1] = pos[j + 2]; posArr[j + 2] = -pos[j + 1];
    if (statusMode) { const c = STATUS_F[s] || STATUS_F[0]; colArr[j] = c.r; colArr[j + 1] = c.g; colArr[j + 2] = c.b; continue; }
    let r = col[j] / 255, g = col[j + 1] / 255, b = col[j + 2] / 255;
    if (r + g + b < 0.02) { const d = showDark ? (pos[j + 2] < 0.5 ? 0.05 : 0.09) : 0; r = g = b = d; }
    colArr[j] = r * r; colArr[j + 1] = g * g; colArr[j + 2] = b * b;   // sRGB-ish -> linear
  }
  dGeom.setDrawRange(0, n);
  dGeom.attributes.position.needsUpdate = true; dGeom.attributes.color.needsUpdate = true;
  if (selectedDrone >= 0 && selectedDrone < n && kind === 2) {
    const j = selectedDrone * 3; selMarker.visible = true;
    selMarker.position.set(posArr[j], posArr[j + 1], posArr[j + 2]); selMarker.lookAt(camera.position);
  } else selMarker.visible = false;
  if (kind === 1) updateTimeUI(t, design ? design.duration : 0);
}

const ray = new THREE.Raycaster(); ray.params.Points.threshold = 2.0;
let downXY = null;
renderer.domElement.addEventListener('pointerdown', (e) => { downXY = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downXY || Math.hypot(e.clientX - downXY[0], e.clientY - downXY[1]) > 4) return;
  const r = renderer.domElement.getBoundingClientRect();
  const m = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  ray.setFromCamera(m, camera);
  const hit = ray.intersectObject(dPoints)[0];
  if (!hit) return;
  if (tab === 'fleet' && lastFrameKind === 2) selectDrone(hit.index);
  else toast(`Show drone #${hit.index + 1}`);
});

let lastTs = performance.now();
function animate(ts) {
  const dt = Math.min(0.1, (ts - lastTs) / 1000); lastTs = ts;
  if (tab === 'design' && design && !design.error && playing) {
    playT += dt * speed;
    if (playT >= design.duration) { playT = design.duration; playing = false; $('#btnPlay').textContent = '▶'; }
    requestPreview(playT);
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

/* ============================================================================
   websocket
   ========================================================================== */
let ws = null;
function connectWS() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => { sendView(); if (design) requestPreview(playT, true); };
  ws.onclose = () => setTimeout(connectWS, 1000);
  ws.onmessage = (ev) => {
    if (typeof ev.data !== 'string') {
      const buf = ev.data; const dv = new DataView(buf);
      const kind = dv.getUint8(0), n = dv.getUint32(4, true), t = dv.getFloat64(8, true);
      const pos = new Float32Array(buf, 16, n * 3);
      const col = new Uint8Array(buf, 16 + n * 12, n * 3);
      const st = new Uint8Array(buf, 16 + n * 15, n);
      if (kind === 1) { inflight = false; if (tab !== 'design') return; }
      if (kind === 2 && tab !== 'fleet') return;
      applyFrame(kind, n, t, pos, col, st);
      if (kind === 1 && Math.abs(wantT - t) > 1e-6) requestPreview(wantT);
      return;
    }
    const msg = JSON.parse(ev.data);
    if (msg.type === 'design') onDesign(msg.design);
    else if (msg.type === 'status') onStatus(msg);
  };
}
function sendView() {
  if (ws && ws.readyState === 1) ws.send(JSON.stringify({ type: 'view', mode: tab === 'fleet' ? 'live' : 'preview' }));
}
function requestPreview(t, force = false) {
  wantT = t;
  if (!ws || ws.readyState !== 1 || (inflight && !force)) return;
  inflight = true;
  ws.send(JSON.stringify({ type: 'preview', t }));
}

/* ============================================================================
   tabs
   ========================================================================== */
$$('.tab').forEach((b) => b.addEventListener('click', () => {
  tab = b.dataset.tab;
  $$('.tab').forEach((x) => x.classList.toggle('active', x === b));
  $$('.pane').forEach((p) => (p.hidden = p.dataset.pane !== tab));
  $('#emergency').hidden = tab !== 'fleet';
  document.body.classList.toggle('fleetmode', tab === 'fleet');
  $('#colorModeWrap').style.display = tab === 'fleet' ? '' : 'none';
  $('#btnPlay').disabled = tab === 'fleet'; $('#scrub').disabled = tab === 'fleet';
  sendView();
  if (tab === 'design') { dGeom.setDrawRange(0, 0); requestPreview(playT, true); refreshLayout(); }
  else { dGeom.setDrawRange(0, 0); refreshLayout(); }
  setTimeout(resize, 0);
}));

/* ============================================================================
   design: show header & settings
   ========================================================================== */
const SETTINGS_GROUPS = [
  ['Safety distances', [['min_separation', 'Min separation (m)'], ['formation_spacing', 'Formation spacing (m)'], ['takeoff_spacing', 'Pad spacing (m)'], ['downwash_factor', 'Downwash factor (1 = sphere)']]],
  ['Speed limits', [['max_speed_h', 'Horizontal (m/s)'], ['max_speed_up', 'Climb (m/s)'], ['max_speed_down', 'Descent (m/s)'], ['max_accel', 'Acceleration (m/s²)']]],
  ['Altitude', [['takeoff_altitude', 'Take-off layer (m)'], ['min_altitude', 'Min formation (m)'], ['max_altitude', 'Ceiling (m)']]],
  ['Field & audience', [['geofence.x', 'Fence x min,max'], ['geofence.y', 'Fence y min,max'], ['geofence.z', 'Fence z min,max'], ['audience_y', 'Audience line y (m)'], ['audience_buffer', 'Audience buffer (m)'], ['takeoff_center', 'Pad grid centre x,y']]],
  ['Location', [['origin.lat', 'Latitude'], ['origin.lon', 'Longitude'], ['origin.alt_amsl', 'Ground alt AMSL (m)'], ['origin.heading_deg', 'Audience looks toward (°)']]],
  ['Battery & lights', [['battery_minutes', 'Battery endurance (min)'], ['battery_reserve_pct', 'Reserve (%)'], ['transition_dim', 'Transition brightness'], ['light_fps', 'Light file rate (fps)'], ['takeoff_color', 'Take-off colour']]],
  ['Live operations', [['deviation_warn', 'Deviation warning (m)'], ['deviation_abort', 'Deviation take-out (m)'], ['min_battery_preflight', 'Min battery to fly (%)'], ['critical_battery', 'Critical battery (%)'], ['require_rtk', 'Require RTK fix'], ['min_satellites', 'Min satellites'], ['max_slot_offset', 'Max pad offset (m)'], ['min_ready_fraction', 'Min ready share (0–1)'], ['max_wind', 'Max wind (m/s)'], ['mission_sync', 'Mission sync'], ['mission_max_items', 'Max mission items (imported)']]],
];
function getPath(o, p) { return p.split('.').reduce((a, k) => (a == null ? undefined : a[k]), o); }
function setPath(o, p, v) { const ks = p.split('.'); let a = o; ks.slice(0, -1).forEach((k) => { a[k] = a[k] ?? {}; a = a[k]; }); a[ks.at(-1)] = v; }

function renderHeader() {
  $('#showName').value = spec.name || '';
  $('#showDrones').value = spec.drones;
  $('#showSpares').value = spec.spares ?? 0;
  const imp = !!spec.imported;
  $('#showDrones').disabled = imp;
  $('#showDrones').title = imp ? 'Fixed by the imported show' : '';
  $('#timelineBlock').hidden = imp;
  if (imp) $('#editor').hidden = true;
  renderImport();
}

/* ============================================================================
   import from Blender / Skybrush Studio
   ========================================================================== */
const FMT_NAMES = { skyc: 'Skybrush .skyc', json: 'Skybrush show.json', csv: 'Skybrush CSV export' };
function renderImport() {
  const imp = spec.imported;
  $('#importBlock').hidden = !imp;
  if (!imp) return;
  $('#impRot').value = imp.rotate_deg ?? 0;
  $('#impOffX').value = (imp.offset || [0, 0])[0];
  $('#impOffY').value = (imp.offset || [0, 0])[1];
  $('#impTol').value = imp.tolerance ?? 0.25;
  const d = design && design.imported;
  const rows = [['File', esc(imp.original_name || '')], ['Format', FMT_NAMES[imp.format] || esc(imp.format)],
    ['Drones', imp.drones], ['Duration', fmtD(imp.duration)]];
  if (d) {
    if (d.first_takeoff != null) rows.push(['Take-offs', `${fmtT(d.first_takeoff)} … ${fmtT(d.last_takeoff)}`]);
    if (d.cues && d.cues.length) rows.push(['Cues', d.cues.length]);
    if (d.has_audio) rows.push(['Music', 'in file - play from the show audio']);
    const L = d.design_limits || {};
    if (L.minDistance) rows.push(['Designed for', `≥ ${L.minDistance} m, ≤ ${L.maxVelocityXY ?? '?'} m/s`]);
  }
  $('#impInfo').innerHTML = rows.map(([k, v]) => `<span class="k">${k}</span><span class="v">${v}</span>`).join('');
  $('#impNotes').innerHTML = (d ? d.notes : []).map((n) => `<li>${esc(n)}</li>`).join('');
}
$('#btnImport').addEventListener('click', () => $('#importFile').click());
$('#importFile').addEventListener('change', (e) => act(async () => {
  const f = e.target.files[0]; e.target.value = '';
  if (!f) return;
  if (spec.timeline && spec.timeline.length && !spec.imported &&
      !confirm(`Import "${f.name}"? The current formation timeline is kept and comes back with "Remove import".`)) return;
  await saveSpec();
  const r = await fetch(`/api/import?name=${encodeURIComponent(f.name)}`, { method: 'POST', body: f, headers: { 'Content-Type': 'application/octet-stream' } });
  let data = null; try { data = await r.json(); } catch { /* empty */ }
  if (!r.ok) throw new Error((data && data.error) || `${r.status} ${r.statusText}`);
  spec = data.spec; selected = -1; design = null; framed = false; dirty = false; $('#dirty').hidden = true;
  renderAll();
  return data;
}, 'Show imported - compiling and validating'));
function impChanged() {
  const imp = spec.imported; if (!imp) return;
  imp.rotate_deg = parseFloat($('#impRot').value) || 0;
  imp.offset = [parseFloat($('#impOffX').value) || 0, parseFloat($('#impOffY').value) || 0];
  imp.tolerance = Math.min(5, Math.max(0.05, parseFloat($('#impTol').value) || 0.25));
  markDirty(); refreshLayoutSoon();
}
['#impRot', '#impOffX', '#impOffY', '#impTol'].forEach((s) => $(s).addEventListener('change', impChanged));
$('#btnImpCenter').addEventListener('click', () => act(async () => {
  impChanged(); await saveSpec();
  spec = await api('/api/import/center', 'POST'); renderImport(); markDirty(); refreshLayoutSoon();
}, 'Centred on the take-off field'));
$('#btnImpRemove').addEventListener('click', () => act(async () => {
  if (!confirm('Remove the imported show and go back to the formation timeline?')) return;
  spec = await api('/api/import/remove', 'POST'); selected = spec.timeline.length ? 0 : -1; design = null;
  renderAll(); await api('/api/compile', 'POST'); dirty = false; $('#dirty').hidden = true;
}, 'Import removed'));
let layoutTimer = null;
function refreshLayoutSoon() { clearTimeout(layoutTimer); layoutTimer = setTimeout(async () => { await saveSpec(); refreshLayout(); }, 500); }
$('#showName').addEventListener('change', (e) => { spec.name = e.target.value; markDirty(); });
$('#showDrones').addEventListener('change', (e) => { spec.drones = Math.max(2, Math.min(10000, parseInt(e.target.value) || 2)); markDirty(); });
$('#showSpares').addEventListener('change', (e) => { spec.spares = Math.max(0, parseInt(e.target.value) || 0); markDirty(); });

function renderSettings() {
  spec.settings = spec.settings || {};
  const d = info.defaults;
  let h = '';
  for (const [title, fields] of SETTINGS_GROUPS) {
    h += `<div class="group">${esc(title)}</div><div class="grid2">`;
    for (const [key, label] of fields) {
      let v = getPath(spec.settings, key); if (v === undefined) v = getPath(d, key);
      if (typeof v === 'boolean') h += `<label><input type="checkbox" data-set="${key}" ${v ? 'checked' : ''}> ${esc(label)}</label>`;
      else if (key === 'mission_sync') h += `<label>${esc(label)}<select data-set="${key}"><option value="hold_time" ${v === 'hold_time' ? 'selected' : ''}>hold times (PX4+AP)</option><option value="utc_delay" ${v === 'utc_delay' ? 'selected' : ''}>UTC resync (ArduPilot)</option></select></label>`;
      else if (key === 'takeoff_color') h += `<label>${esc(label)}<input type="color" data-set="${key}" value="${esc(v)}"></label>`;
      else if (Array.isArray(v)) h += `<label>${esc(label)}<input type="text" data-set="${key}" data-arr="1" value="${v.join(', ')}"></label>`;
      else h += `<label>${esc(label)}<input type="number" step="any" data-set="${key}" value="${v}"></label>`;
    }
    h += '</div>';
  }
  $('#settingsForm').innerHTML = h;
  $$('#settingsForm [data-set]').forEach((el) => el.addEventListener('change', () => {
    const k = el.dataset.set; let v;
    if (el.type === 'checkbox') v = el.checked;
    else if (el.dataset.arr) v = el.value.split(',').map((x) => parseFloat(x)).filter((x) => !isNaN(x));
    else if (el.type === 'number') v = parseFloat(el.value);
    else v = el.value;
    if (k.includes('.')) {
      const root = k.split('.')[0];
      if (!spec.settings[root]) spec.settings[root] = JSON.parse(JSON.stringify(info.defaults[root]));
    }
    setPath(spec.settings, k, v); markDirty(); refreshLayout();
  }));
}

/* ============================================================================
   design: timeline + formation editor
   ========================================================================== */
const SHAPE_PARAMS = {
  text: [['text', 'Text (new line = Enter)', 'textarea', 'HELLO'], ['style', 'Style', 'select:fill,outline', 'fill'], ['font', 'Font file (optional, e.g. arialbd.ttf)', 'text', '']],
  image: [['image', 'Image (PNG with transparent background works best)', 'image', ''], ['threshold', 'Brightness threshold (no alpha)', 'number', 0.5], ['invert', 'Use dark pixels', 'bool', false]],
  disk: [], heart: [], ring: [['inner', 'Inner radius ratio', 'number', 0.6]],
  star: [['points', 'Points', 'number', 5], ['inner', 'Inner ratio', 'number', 0.42]],
  wall: [['aspect', 'Width / height', 'number', 1.6]],
  sphere: [], cube: [], torus: [['ratio', 'Tube ratio', 'number', 0.33]],
  cylinder: [['aspect', 'Height ratio', 'number', 1.2]], cone: [['aspect', 'Height ratio', 'number', 1.0]],
  wave: [['amplitude', 'Amplitude (m, blank = auto)', 'number', ''], ['wavelength', 'Wavelength (m, blank = auto)', 'number', '']],
  points: [['csv', 'Points: x,y,z[,r,g,b] per line', 'textarea', ''], ['absolute', 'Use coordinates as given (no auto scaling)', 'bool', false]],
};
const IS_2D = ['text', 'image', 'disk', 'ring', 'heart', 'star', 'wall'];
const EFFECT_FIELDS = {
  solid: ['color'], image: ['color'], gradient: ['color', 'color2', 'axis'], split: ['color', 'color2', 'axis', 'ripple'],
  rainbow: ['axis', 'speed', 'cycles'], pulse: ['color', 'rate', 'low'], twinkle: ['color', 'density', 'rate'],
  wave: ['color', 'color2', 'axis', 'speed', 'cycles'], off: [],
};
const LIGHT_LABEL = { color: 'Colour', color2: 'Colour 2', axis: 'Axis', speed: 'Speed', cycles: 'Cycles', rate: 'Rate (Hz)', low: 'Low level', density: 'Density', ripple: 'Ripple' };
const LIGHT_DEF = { color: '#ffffff', color2: '#ff0000', axis: 'x', speed: 0.3, cycles: 1, rate: 1, low: 0.3, density: 0.15, ripple: 0 };

function swatch(f) {
  const l = f.light || {};
  if (l.effect === 'rainbow') return 'conic-gradient(red, yellow, lime, cyan, blue, magenta, red)';
  if (['gradient', 'split', 'wave'].includes(l.effect)) return `linear-gradient(90deg, ${l.color || '#fff'}, ${l.color2 || '#f00'})`;
  return l.color || '#fff';
}
function renderTimeline() {
  const tl = spec.timeline;
  $('#tlCount').textContent = `${tl.length} formations`;
  $('#timeline').innerHTML = tl.map((f, i) => `
    <li data-i="${i}" class="${i === selected ? 'sel' : ''}">
      <span class="sw" style="background:${swatch(f)}"></span>
      <span class="nm">${i + 1}. ${esc(f.name || f.shape)}</span>
      <span class="meta">${esc(f.shape)} · ${countLabel(f.count)} · ${f.hold ?? 5}s</span>
      <button data-mv="-1" title="Move up">↑</button><button data-mv="1" title="Move down">↓</button>
      <button data-dup title="Duplicate">⧉</button><button data-del title="Delete">✕</button>
    </li>`).join('');
  $$('#timeline li').forEach((li) => li.addEventListener('click', (e) => {
    const i = +li.dataset.i; const b = e.target.closest('button');
    if (b) {
      e.stopPropagation();
      if (b.dataset.mv) { const j = i + +b.dataset.mv; if (j < 0 || j >= tl.length) return; [tl[i], tl[j]] = [tl[j], tl[i]]; selected = j; }
      else if (b.hasAttribute('data-dup')) { tl.splice(i + 1, 0, JSON.parse(JSON.stringify(tl[i]))); selected = i + 1; }
      else if (b.hasAttribute('data-del')) { if (tl.length <= 1) return toast('A show needs at least one formation', true); tl.splice(i, 1); selected = Math.min(selected, tl.length - 1); }
      markDirty(); renderTimeline(); renderEditor(); return;
    }
    selected = i; renderTimeline(); renderEditor(); jumpToFormation(i);
  }));
}
function countLabel(c) { c = c ?? 1; return c <= 1 ? `${Math.round(c * 100)}%` : `${c}`; }
function jumpToFormation(i) {
  if (!design || design.error || !design.formation_start) return;
  const t = design.formation_start[String(i)];
  if (t !== undefined) { playT = t + 0.5; requestPreview(playT); }
}
$('#btnAdd').addEventListener('click', () => {
  spec.timeline.push({ name: `Formation ${spec.timeline.length + 1}`, shape: 'text', params: { text: 'HELLO', style: 'fill' }, count: 0.4,
    center: [0, 0, 80], rotation: [0, 0, 0], plane: 'vertical', transition: 0, hold: 6, light: { effect: 'solid', color: '#ffffff' } });
  selected = spec.timeline.length - 1; markDirty(); renderTimeline(); renderEditor();
});

function field(label, html) { return `<label>${esc(label)}${html}</label>`; }
function renderEditor() {
  const f = spec.timeline[selected];
  $('#editor').hidden = !f;
  if (!f) return;
  f.params = f.params || {}; f.light = f.light || { effect: 'solid', color: '#ffffff' }; f.spin = f.spin || { angle: 0, axis: 'z' };
  f.center = f.center || [0, 0, 80]; f.rotation = f.rotation || [0, 0, 0];
  $('#edIndex').textContent = `#${selected + 1}`;
  const shapes = info.shapes.map((s) => `<option ${s === f.shape ? 'selected' : ''}>${s}</option>`).join('');
  let h = `<div class="row"><label class="grow">Name<input data-f="name" value="${esc(f.name || '')}"></label>
           <label>Shape<select data-f="shape">${shapes}</select></label></div>`;
  for (const [k, label, type, def] of SHAPE_PARAMS[f.shape] || []) {
    const v = f.params[k] ?? def;
    if (type === 'textarea') h += field(label, `<textarea data-p="${k}" rows="3">${esc(v)}</textarea>`);
    else if (type === 'bool') h += `<label><input type="checkbox" data-p="${k}" ${v ? 'checked' : ''}> ${esc(label)}</label>`;
    else if (type.startsWith('select:')) h += field(label, `<select data-p="${k}">${type.slice(7).split(',').map((o) => `<option ${o === v ? 'selected' : ''}>${o}</option>`).join('')}</select>`);
    else if (type === 'image') h += field(label, `<input type="file" accept="image/*" data-img="${k}">${v ? `<img class="thumb" src="${esc(v)}">` : ''}`);
    else h += field(label, `<input type="${type}" step="any" data-p="${k}" value="${esc(v)}">`);
  }
  const sizeKey = IS_2D.includes(f.shape) ? 'height' : 'size';
  h += `<div class="grid2">${field('Drones used (≤1 = share)', `<input type="number" step="any" data-f="count" value="${f.count ?? 1}">`)}
        ${field(IS_2D.includes(f.shape) ? 'Height (m, blank = auto)' : 'Size (m, blank = auto)', `<input type="number" step="any" data-p="${sizeKey}" value="${f.params[sizeKey] ?? ''}">`)}</div>`;
  if (IS_2D.includes(f.shape)) h += field('Plane', `<select data-f="plane"><option value="vertical" ${f.plane !== 'horizontal' ? 'selected' : ''}>vertical (faces audience)</option><option value="horizontal" ${f.plane === 'horizontal' ? 'selected' : ''}>horizontal (seen from below)</option></select>`);
  h += `<div class="group">Position (x right, y depth, z up) &amp; rotation</div><div class="grid3">
        ${['x', 'y', 'z'].map((a, i) => field(a + ' (m)', `<input type="number" step="any" data-c="${i}" value="${f.center[i]}">`)).join('')}
        ${['yaw', 'pitch', 'roll'].map((a, i) => field(a + ' (°)', `<input type="number" step="any" data-r="${i}" value="${f.rotation[i] ?? 0}">`)).join('')}</div>`;
  h += `<div class="group">Timing</div><div class="grid3">
        ${field('Transition (s, 0 = fastest safe)', `<input type="number" step="any" min="0" data-f="transition" value="${f.transition ?? 0}">`)}
        ${field('Hold (s)', `<input type="number" step="any" min="0" data-f="hold" value="${f.hold ?? 5}">`)}
        ${field('Spin (°)', `<input type="number" step="any" data-s="angle" value="${f.spin.angle ?? 0}">`)}</div>
        ${field('Spin axis', `<select data-s="axis">${['z', 'y', 'x'].map((a) => `<option ${a === (f.spin.axis || 'z') ? 'selected' : ''}>${a}</option>`).join('')}</select>`)}`;
  const L = f.light;
  h += `<div class="group">Lights</div>` + field('Effect', `<select data-l="effect">${info.effects.map((e) => `<option ${e === L.effect ? 'selected' : ''}>${e}</option>`).join('')}</select>`);
  h += '<div class="grid2">';
  for (const k of EFFECT_FIELDS[L.effect] || []) {
    const v = L[k] ?? LIGHT_DEF[k];
    if (k.startsWith('color')) h += field(LIGHT_LABEL[k], `<input type="color" data-l="${k}" value="${esc(v)}">`);
    else if (k === 'axis') h += field('Axis', `<select data-l="axis">${['x', 'y', 'z', 'radial'].map((a) => `<option ${a === v ? 'selected' : ''}>${a}</option>`).join('')}</select>`);
    else h += field(LIGHT_LABEL[k], `<input type="number" step="any" data-l="${k}" value="${v}">`);
  }
  h += field('Brightness', `<input type="number" step="0.05" min="0" max="1" data-l="brightness" value="${L.brightness ?? 1}">`) + '</div>';
  $('#edForm').innerHTML = h;

  const num = (el) => (el.value === '' ? '' : parseFloat(el.value));
  $$('#edForm [data-f]').forEach((el) => el.addEventListener('change', () => {
    const k = el.dataset.f;
    f[k] = ['count', 'transition', 'hold'].includes(k) ? (num(el) === '' ? 0 : num(el)) : el.value;
    if (k === 'shape') f.params = {};
    markDirty(); renderTimeline(); if (k === 'shape') renderEditor();
  }));
  $$('#edForm [data-p]').forEach((el) => el.addEventListener('change', () => {
    const k = el.dataset.p;
    if (el.type === 'checkbox') f.params[k] = el.checked;
    else if (el.type === 'number') { const v = num(el); if (v === '') delete f.params[k]; else f.params[k] = v; }
    else f.params[k] = el.value;
    markDirty();
  }));
  $$('#edForm [data-img]').forEach((el) => el.addEventListener('change', () => {
    const file = el.files[0]; if (!file) return;
    if (file.size > 8e6) return toast('Image is larger than 8 MB', true);
    const rd = new FileReader();
    rd.onload = () => { f.params[el.dataset.img] = rd.result; if (!f.light || f.light.effect === 'solid') f.light = { effect: 'image', color: '#ffffff' }; markDirty(); renderEditor(); };
    rd.readAsDataURL(file);
  }));
  $$('#edForm [data-c]').forEach((el) => el.addEventListener('change', () => { f.center[+el.dataset.c] = num(el) || 0; markDirty(); }));
  $$('#edForm [data-r]').forEach((el) => el.addEventListener('change', () => { f.rotation[+el.dataset.r] = num(el) || 0; markDirty(); }));
  $$('#edForm [data-s]').forEach((el) => el.addEventListener('change', () => { f.spin[el.dataset.s] = el.dataset.s === 'angle' ? (num(el) || 0) : el.value; markDirty(); }));
  $$('#edForm [data-l]').forEach((el) => el.addEventListener('change', () => {
    const k = el.dataset.l; f.light[k] = el.type === 'number' ? num(el) : el.value;
    markDirty(); renderTimeline(); if (k === 'effect') renderEditor();
  }));
}

/* ============================================================================
   design: save / compile / export
   ========================================================================== */
function markDirty() {
  dirty = true; $('#dirty').hidden = false;
  clearTimeout(saveTimer); saveTimer = setTimeout(() => saveSpec().catch((e) => toast(e.message, true)), 400);
}
async function saveSpec() { clearTimeout(saveTimer); await api('/api/spec', 'PUT', spec); }
$('#btnCompile').addEventListener('click', () => act(async () => { await saveSpec(); await api('/api/compile', 'POST'); dirty = false; $('#dirty').hidden = true; }));
$('#btnExport').addEventListener('click', () => act(async () => {
  if (dirty) throw new Error('Compile the latest changes before exporting');
  $('#exportBlock').hidden = false; $('#exportInfo').innerHTML = '<p class="muted">Writing per-drone files…</p>';
  const r = await api('/api/export', 'POST', { include: ['trajectories', 'missions', 'lights'] });
  $('#exportInfo').innerHTML = `<p>Saved <b>${r.drones}</b> drone programmes:</p><p class="mono hint">${esc(r.path)}</p>
     <p><a href="${r.url}" download>Download .zip</a></p>`;
}, 'Export complete'));
$('#btnNew').addEventListener('click', () => act(async () => {
  if (!confirm('Replace the current show with the demo show?')) return;
  spec = await api('/api/spec/new', 'POST', { drones: parseInt($('#showDrones').value) || 3000 });
  selected = 0; renderAll(); await api('/api/compile', 'POST'); dirty = false; $('#dirty').hidden = true;
}));
$('#btnSave').addEventListener('click', () => act(async () => {
  const name = prompt('Save show as', spec.name || 'show'); if (!name) return;
  await saveSpec(); const r = await api('/api/shows/save', 'POST', { name }); refreshShowList(); return r;
}, 'Show saved'));
$('#loadSelect').addEventListener('change', (e) => act(async () => {
  const name = e.target.value; e.target.value = ''; if (!name) return;
  spec = await api('/api/shows/load', 'POST', { name }); selected = 0; renderAll();
  await api('/api/compile', 'POST'); dirty = false; $('#dirty').hidden = true;
}, 'Show loaded'));
async function refreshShowList() {
  const r = await api('/api/shows');
  $('#loadSelect').innerHTML = '<option value="">Load…</option>' + r.shows.map((s) => `<option>${esc(s)}</option>`).join('');
}

function onDesign(d) {
  design = d;
  if (spec && spec.imported) renderImport();
  const vEl = $('#validation');
  if (!d) { vEl.innerHTML = '<p class="muted">Not compiled yet.</p>'; return; }
  if (d.error) { vEl.innerHTML = `<div class="verdict bad">Compile error</div><p>${esc(d.error)}</p>`; return; }
  const v = d.validation;
  vEl.innerHTML = `<div class="verdict ${v.ok ? 'ok' : 'bad'}">${v.ok ? '✓ Safe to fly (all checks passed)' : '✕ Not safe — fix the failing checks'}</div>` +
    v.checks.map((c) => `<div class="ck ${c.status}"><span class="ic">${c.status === 'pass' ? '✓' : c.status === 'warn' ? '!' : '✕'}</span>
      <div><div class="nm">${esc(c.name)}</div><div class="dt">${esc(c.detail)}</div></div></div>`).join('') +
    (v.violations.length ? `<div class="hint">First conflicts: ${v.violations.slice(0, 6).map((x) => `<a href="#" data-t="${x.t}" style="color:#ff8aa0">t=${x.t}s #${x.a + 1}/#${x.b + 1} ${x.dist}m</a>`).join(', ')}</div>` : '');
  $$('#validation [data-t]').forEach((a) => a.addEventListener('click', (e) => { e.preventDefault(); playT = +a.dataset.t; requestPreview(playT); }));
  const s = v.stats;
  $('#stats').innerHTML = [
    ['Drones', s.drones], ['Duration', fmtD(s.duration_s)], ['Closest approach', s.min_separation_m == null ? '> ' + (1.25 * d.settings.min_separation).toFixed(2) + ' m' : s.min_separation_m + ' m'],
    ['Peak speed h / up / down', `${s.max_speed_h} / ${s.max_speed_up} / ${s.max_speed_down} m/s`], ['Peak accel', `${s.max_accel} m/s²`],
    ['Extent x', `${s.extent.x[0]} … ${s.extent.x[1]} m`], ['Extent y', `${s.extent.y[0]} … ${s.extent.y[1]} m`], ['Height', `${s.extent.z[1]} m`],
  ].map(([k, val]) => `<span class="k">${k}</span><span class="v">${val}</span>`).join('');
  $('#segments').innerHTML = d.segments.map((sg) => `<li data-t="${sg.t0}"><span class="t">${fmtD(sg.t0)}</span><span class="kd">${sg.kind}</span><span>${esc(sg.label)}</span></li>`).join('');
  $$('#segments li').forEach((li) => li.addEventListener('click', () => { playT = +li.dataset.t + 0.05; requestPreview(playT); }));
  $('#segStrip').innerHTML = d.segments.map((sg) => `<div style="flex:${Math.max(0.001, sg.t1 - sg.t0)};background:${KIND_COLORS[sg.kind]}" title="${esc(sg.label)}"></div>`).join('');
  $('#scrub').max = d.duration;
  playT = Math.min(playT, d.duration);
  if (!framed) { framed = true; setCam('audience'); }
  if (tab === 'design') requestPreview(playT, true);
}
function updateTimeUI(t, dur) {
  if (tab !== 'design') return;
  $('#scrub').value = t;
  $('#timeLabel').textContent = `${fmtT(t)} / ${fmtD(dur)}`;
  if (design && design.segments) {
    const sg = design.segments.find((x) => t >= x.t0 && t < x.t1) || design.segments.at(-1);
    $('#hud').textContent = `${sg ? sg.label : ''}\n${design.drones} drones`;
  }
}
$('#btnPlay').addEventListener('click', () => {
  if (!design || design.error) return;
  playing = !playing; if (playing && playT >= design.duration - 0.05) playT = 0;
  $('#btnPlay').textContent = playing ? '❚❚' : '▶';
});
$('#speed').addEventListener('change', (e) => (speed = parseFloat(e.target.value)));
$('#scrub').addEventListener('input', (e) => { playT = parseFloat(e.target.value); requestPreview(playT); });
$('#showDark').addEventListener('change', () => { if (tab === 'design') requestPreview(playT, true); });

/* ============================================================================
   fleet & operations
   ========================================================================== */
$('#connKind').addEventListener('change', (e) => { $('#simOpts').hidden = e.target.value !== 'sim'; $('#mavOpts').hidden = e.target.value !== 'mavlink'; });
$('#btnConnect').addEventListener('click', () => act(async () => {
  await saveSpec();
  const kind = $('#connKind').value;
  const body = { kind, drones: spec.drones, spares: spec.spares ?? 0 };
  if (kind === 'sim') { body.preflight_fault_rate = (parseFloat($('#simPre').value) || 0) / 100; body.inflight_fault_per_min = (parseFloat($('#simIn').value) || 0) / 1000; }
  else body.endpoints = $('#mavEndpoints').value.split('\n').map((x) => x.trim()).filter(Boolean);
  await api('/api/fleet/connect', 'POST', body); await refreshLayout(); sendView();
}, 'Fleet connected'));
$('#btnDisconnect').addEventListener('click', () => act(() => api('/api/fleet/disconnect', 'POST'), 'Disconnected'));
const op = (a, body, msg) => act(async () => {
  try { return await api(`/api/op/${a}`, 'POST', body); }
  catch (e) {
    if (!e.confirm) throw e;
    if (!confirm(`${e.message}\n\nDo it anyway?`)) return null;
    return api(`/api/op/${a}`, 'POST', { ...(body || {}), force: true });
  }
}, msg);
$('#opPlan').addEventListener('click', () => op('plan', undefined, 'Plan ready'));
$('#opUpload').addEventListener('click', () => op('upload', undefined, 'Show uploaded'));
$('#opStart').addEventListener('click', () => {
  if (!confirm('Start the countdown? Drones will arm and take off when it reaches zero.')) return;
  op('start', { countdown: parseFloat($('#countdown').value) || 30 }, 'Countdown started');
});
$('#opAbort').addEventListener('click', () => op('abort', undefined, 'Countdown aborted'));
$('#wind').addEventListener('change', (e) => op('wind', { value: e.target.value }));
$('#audAction').addEventListener('change', (e) => op('audience_action', { value: e.target.value }));
$('#emHold').addEventListener('click', () => op('hold'));
$('#emResume').addEventListener('click', () => op('resume'));
$('#emReturn').addEventListener('click', () => op('return', undefined, 'Coordinated return started'));
$('#emLand').addEventListener('click', () => { if (confirm('Land every drone straight down where it is? Drones above others may descend onto them — prefer COORDINATED RETURN.')) op('land_all'); });
$('#emKill').addEventListener('click', () => { $('#killText').value = ''; $('#killDlg').showModal(); });
$('#killDlg').addEventListener('close', () => { if ($('#killDlg').returnValue === 'ok') op('kill_all', { confirm: $('#killText').value.trim() }); });
$$('[data-dact]').forEach((b) => b.addEventListener('click', () => {
  if (selectedDrone < 0) return;
  const a = b.dataset.dact;
  if (a === 'kill' && !confirm('Stop the motors of this drone? It will fall.')) return;
  act(() => api(`/api/fleet/drone/${selectedDrone}/${a}`, 'POST'), 'Command sent');
}));

async function refreshLayout() {
  try { layoutData = await api('/api/layout'); buildField(layoutData.settings, layoutData); } catch { /* ignore */ }
}

const ACTIVE = ['COUNTDOWN', 'RUNNING', 'HOLDING', 'RETURNING', 'LANDING'];
function onStatus(msg) {
  const s = msg.status; status = s;
  // job progress (design compile/export)
  const j = s.job;
  const cp = $('#compileProgress');
  cp.hidden = !(j && j.running);
  if (j && j.running) { cp.firstElementChild.style.width = `${Math.round(j.frac * 100)}%`; cp.lastElementChild.textContent = j.msg; }
  $('#btnCompile').disabled = !!(j && j.running);
  // top bar
  const chip = $('#stateChip');
  chip.textContent = s.state;
  chip.className = 'chip ' + ({ RUNNING: 'live', COUNTDOWN: 'live', FINISHED: 'ok', UPLOADED: 'ok', PLANNED: 'ok' }[s.state] || '') +
    (['HOLDING', 'RETURNING', 'LANDING', 'ABORTED'].includes(s.state) ? ' alarm' : '');
  if (s.state === 'COUNTDOWN') $('#clock').textContent = `T− ${fmtT(s.countdown)}`;
  else if (s.t_show !== undefined) $('#clock').textContent = `T+ ${fmtT(s.t_show)}`;
  const f = s.fleet;
  $('#fleetSummary').textContent = f ? `${f.kind === 'sim' ? 'SIM' : 'MAVLink'} · ${f.known} drones · ${f.counts.flying + f.counts.warning + f.counts.failsafe} airborne` : '';
  // operations panel
  const st = s.state;
  const has = !!f;
  $('#btnConnect').disabled = ACTIVE.includes(st); $('#btnDisconnect').disabled = !has || ACTIVE.includes(st);
  $('#connKind').disabled = has;
  $('#opPlan').disabled = !['CONNECTED', 'PLANNED', 'UPLOADED', 'FINISHED', 'ABORTED'].includes(st);
  $('#opUpload').disabled = !['PLANNED', 'UPLOADED'].includes(st);
  $('#opStart').disabled = !(st === 'UPLOADED' && s.gonogo && s.gonogo.go);
  $('#opAbort').disabled = st !== 'COUNTDOWN';
  $('#emHold').disabled = st !== 'RUNNING'; $('#emResume').disabled = st !== 'HOLDING';
  $('#emReturn').disabled = !['RUNNING', 'HOLDING'].includes(st);
  $('#emLand').disabled = !ACTIVE.includes(st); $('#emKill').disabled = !has || !ACTIVE.includes(st);
  const pr = $('#opProgress'); const p = s.progress;
  pr.hidden = !(p && p.task);
  if (p && p.task) { pr.firstElementChild.style.width = `${Math.round(p.frac * 100)}%`; pr.lastElementChild.textContent = p.msg; }
  renderChecklist(s);
  const g = s.gonogo, gEl = $('#gonogo');
  if (g) {
    const rs = Object.entries(g.reasons || {}).map(([k, v]) => `${v}× ${k}`).join(', ');
    gEl.className = 'gonogo ' + (g.go ? 'go' : 'nogo');
    gEl.innerHTML = `<b>${g.go ? 'GO' : 'NO-GO'}</b> — ${g.ready}/${g.flyers} drones ready (need ${g.required})` +
      `${rs ? `<br><span class="muted">${esc(rs)}</span>` : ''}<br>Checklist ${g.checklist_ok ? '✓' : '✕'} · Wind ${g.wind_ok ? '✓' : '✕'} (limit ${g.max_wind} m/s)`;
  } else { gEl.className = 'gonogo'; gEl.textContent = 'Plan and upload the show to evaluate.'; }
  if ($('#audAction').value !== s.audience_action) $('#audAction').value = s.audience_action;
  // counts + grid
  if (f) {
    $('#counts').innerHTML = STATUS_LABEL.map((n, i) => `<div><i style="background:${STATUS_RGB[i]}"></i>${n}<b>${f.counts[n]}</b></div>`).join('');
    if (msg.grid) drawGrid(Uint8Array.from(atob(msg.grid), (c) => c.charCodeAt(0)));
    if (tab === 'fleet') {
      $('#hud').textContent = `${s.state}${s.return_phase ? ' · return phase ' + s.return_phase : ''}\n` +
        `${f.flyers} flyers · ${f.taken_out} out of show${f.min_batt != null ? ` · min battery ${f.min_batt.toFixed(0)}%` : ''}`;
      const dur = s.show ? s.show.duration : 0;
      $('#timeLabel').textContent = `${fmtT(s.t_show || 0)} / ${fmtD(dur)}`;
      $('#scrub').max = dur || 1; $('#scrub').value = Math.min(dur, s.t_show || 0);
    }
  } else { $('#counts').innerHTML = '<span class="muted">No fleet connected.</span>'; drawGrid(new Uint8Array(0)); }
  if (msg.events) addEvents(msg.events);
}
function renderChecklist(s) {
  const el = $('#checklist');
  if (!el.dataset.built) {
    el.innerHTML = s.checklist_items.map(([k, label]) => `<label><input type="checkbox" data-ck="${k}"> ${esc(label)}</label>`).join('');
    $$('#checklist [data-ck]').forEach((cb) => cb.addEventListener('change', () => op('checklist', { key: cb.dataset.ck, value: cb.checked })));
    el.dataset.built = '1';
  }
  $$('#checklist [data-ck]').forEach((cb) => { const v = !!s.checklist[cb.dataset.ck]; if (cb.checked !== v && document.activeElement !== cb) cb.checked = v; });
  const w = $('#wind'); if (document.activeElement !== w) w.value = s.wind ?? '';
}
let gridCols = 1, gridCell = 1, gridN = 0;
function drawGrid(arr) {
  const cv = $('#grid'); const W = cv.clientWidth || 290;
  gridN = arr.length;
  gridCols = Math.max(1, Math.ceil(Math.sqrt(gridN * 1.6)));
  gridCell = Math.max(2, Math.floor(W / gridCols));
  const rows = Math.ceil(gridN / gridCols);
  cv.width = gridCols * gridCell; cv.height = Math.max(20, rows * gridCell);
  const g = cv.getContext('2d');
  g.fillStyle = '#05070d'; g.fillRect(0, 0, cv.width, cv.height);
  for (let i = 0; i < gridN; i++) {
    g.fillStyle = STATUS_RGB[arr[i]] || '#333';
    g.fillRect((i % gridCols) * gridCell, Math.floor(i / gridCols) * gridCell, gridCell - (gridCell > 3 ? 1 : 0), gridCell - (gridCell > 3 ? 1 : 0));
  }
  if (selectedDrone >= 0 && selectedDrone < gridN) {
    g.strokeStyle = '#fff'; g.lineWidth = 1;
    g.strokeRect((selectedDrone % gridCols) * gridCell - 1, Math.floor(selectedDrone / gridCols) * gridCell - 1, gridCell + 1, gridCell + 1);
  }
}
function gridIndex(e) {
  const cv = $('#grid'); const r = cv.getBoundingClientRect();
  const x = (e.clientX - r.left) * (cv.width / r.width), y = (e.clientY - r.top) * (cv.height / r.height);
  const i = Math.floor(y / gridCell) * gridCols + Math.floor(x / gridCell);
  return i >= 0 && i < gridN ? i : -1;
}
$('#grid').addEventListener('mousemove', (e) => { const i = gridIndex(e); $('#gridTip').textContent = i >= 0 ? `drone #${i + 1} · ${STATUS_LABEL[lastStatus[i]] || ''}` : ''; });
$('#grid').addEventListener('click', (e) => { const i = gridIndex(e); if (i >= 0) selectDrone(i); });

let droneTimer = null;
async function selectDrone(i) {
  selectedDrone = i; $('#droneBlock').hidden = false;
  clearInterval(droneTimer);
  const load = async () => {
    try {
      const d = await api(`/api/fleet/drone/${i}`);
      $('#droneTitle').textContent = `${d.id}`;
      const rows = [['Status', d.status], ['Flags', d.flags || '—'], ['Role', d.role], ['Mode', d.mode], ['Autopilot', d.autopilot],
        ['Battery', `${d.battery}% · ${d.voltage} V`], ['GPS', `fix ${d.gps_fix} · ${d.sats} sats`], ['Position', d.pos.map((v) => v ?? '–').join(', ')],
        ['Pad / show slot', `${d.pad} / ${d.slot}`], ['Pad offset', d.pad_offset == null ? '–' : d.pad_offset + ' m'], ['Show uploaded', d.uploaded ? 'yes' : 'no'],
        ['Link age', d.link_age + ' s']];
      $('#droneInfo').innerHTML = rows.map(([k, v]) => `<span class="k">${k}</span><span class="v">${esc(v)}</span>`).join('');
    } catch (e) { $('#droneInfo').textContent = e.message; clearInterval(droneTimer); }
  };
  await load(); droneTimer = setInterval(load, 1000);
}
const evList = [];
function addEvents(items) {
  const el = $('#events');
  for (const e of items) {
    const li = document.createElement('li');
    const tm = new Date(e.t * 1000).toLocaleTimeString([], { hour12: false });
    li.innerHTML = `<span class="tm">${tm}</span>${e.drone ? `<span class="dr" data-i="${e.idx}">${esc(e.drone)}</span>` : ''}<span class="${e.level}">${esc(e.msg)}</span>`;
    el.prepend(li); evList.push(li);
    if (evList.length > 300) evList.shift().remove();
  }
  $$('#events .dr').forEach((d) => (d.onclick = () => selectDrone(+d.dataset.i)));
}

/* ============================================================================
   boot
   ========================================================================== */
function renderAll() { renderHeader(); renderSettings(); renderTimeline(); renderEditor(); refreshLayout(); }

/* ---- brand: splash, about, logo on the 3D view ---- */
const splashMsg = (m) => { const el = $('#splashMsg'); if (el) el.textContent = m; };
let splashShown = performance.now();
function hideSplash() {
  const wait = Math.max(0, 1200 - (performance.now() - splashShown));     // let the logo be seen
  setTimeout(() => $('#splash').classList.add('done'), wait);
}
$('#brandBtn').addEventListener('click', () => {
  $('#aboutVer').textContent = info ? `Version ${info.version}` : '';
  $('#aboutHome').textContent = info ? info.home : '';
  $('#aboutDlg').showModal();
});
function setMark(on) {
  $('#viewMark').classList.toggle('off', !on); $('#showMark').checked = on;
  try { localStorage.setItem('ls.mark', on ? '1' : '0'); } catch { /* storage unavailable */ }
}
$('#showMark').addEventListener('change', (e) => setMark(e.target.checked));
try { setMark(localStorage.getItem('ls.mark') !== '0'); } catch { setMark(true); }

async function boot() {
  splashMsg('Connecting to the show engine…');
  info = await api('/api/info');
  $('#ver').textContent = 'v' + info.version;
  splashMsg('Loading the show…');
  spec = await api('/api/spec');
  selected = spec.timeline.length ? 0 : -1;
  renderAll(); refreshShowList();
  $('#colorModeWrap').style.display = 'none';
  resize(); setCam('audience');
  connectWS();
  requestAnimationFrame(animate);
  hideSplash();
  window.__lsReady = true;
  if (window.__lsReport) window.__lsReport('ready');
}
$('#camAudience').addEventListener('click', () => setCam('audience'));
$('#camTop').addEventListener('click', () => setCam('top'));
$('#camSide').addEventListener('click', () => setCam('side'));
boot().catch((e) => {
  const msg = 'Start-up failed: ' + (e && e.message ? e.message : e);
  if (window.__lsReport) window.__lsReport('error', msg + (e && e.stack ? '\n' + e.stack : ''));
  if (window.__lsShowErr) window.__lsShowErr(msg); else splashMsg(msg);
  toast(msg, true);
});
