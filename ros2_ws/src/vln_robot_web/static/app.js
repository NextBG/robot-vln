'use strict';
const $ = (id) => document.getElementById(id);
const S = { state: null, estop: false, armed: false, keys: new Set(), joy: [0, 0], sending: false, rtt: null };

// ---------------------------------------------------------------- coordinates (WGS-84 -> GCJ-02 for AMap tiles)
function outOfChina(lat, lon) { return lon < 72.004 || lon > 137.8347 || lat < 0.8293 || lat > 55.8271; }
function tLat(x, y) {
  let r = -100 + 2 * x + 3 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * Math.sqrt(Math.abs(x));
  r += (20 * Math.sin(6 * x * Math.PI) + 20 * Math.sin(2 * x * Math.PI)) * 2 / 3;
  r += (20 * Math.sin(y * Math.PI) + 40 * Math.sin(y / 3 * Math.PI)) * 2 / 3;
  r += (160 * Math.sin(y / 12 * Math.PI) + 320 * Math.sin(y * Math.PI / 30)) * 2 / 3;
  return r;
}
function tLon(x, y) {
  let r = 300 + x + 2 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * Math.sqrt(Math.abs(x));
  r += (20 * Math.sin(6 * x * Math.PI) + 20 * Math.sin(2 * x * Math.PI)) * 2 / 3;
  r += (20 * Math.sin(x * Math.PI) + 40 * Math.sin(x / 3 * Math.PI)) * 2 / 3;
  r += (150 * Math.sin(x / 12 * Math.PI) + 300 * Math.sin(x / 30 * Math.PI)) * 2 / 3;
  return r;
}
function wgs2gcj(lat, lon) {
  if (outOfChina(lat, lon)) return [lat, lon];
  const a = 6378245.0, ee = 0.00669342162296594323;
  let dLat = tLat(lon - 105, lat - 35), dLon = tLon(lon - 105, lat - 35);
  const rad = lat / 180 * Math.PI;
  let magic = Math.sin(rad); magic = 1 - ee * magic * magic;
  const sq = Math.sqrt(magic);
  dLat = (dLat * 180) / ((a * (1 - ee)) / (magic * sq) * Math.PI);
  dLon = (dLon * 180) / (a / sq * Math.cos(rad) * Math.PI);
  return [lat + dLat, lon + dLon];
}
function gcj2wgs(lat, lon) {
  let w = [lat, lon];
  for (let i = 0; i < 4; i++) { const g = wgs2gcj(w[0], w[1]); w = [w[0] - (g[0] - lat), w[1] - (g[1] - lon)]; }
  return w;
}

// ---------------------------------------------------------------- API
async function post(path, body) {
  const t0 = performance.now();
  const r = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                body: JSON.stringify(body || {}), credentials: 'same-origin' });
  S.rtt = Math.round(performance.now() - t0);
  if (r.status === 401) location.href = '/';
  return r.json().catch(() => ({}));
}

// ---------------------------------------------------------------- map
const amapOpt = { subdomains: '1234', maxZoom: 19, maxNativeZoom: 18, attribution: '高德地图' };
const gcjLayer = (layer) => { layer.gcj = true; return layer; };
const baseLayers = {
  '卫星 + 路网': gcjLayer(L.layerGroup([
    L.tileLayer('https://webst0{s}.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}', amapOpt),
    // transparent road + label overlay
    L.tileLayer('https://webst0{s}.is.autonavi.com/appmaptile?style=8&x={x}&y={y}&z={z}', amapOpt),
  ])),
  '高德街道': gcjLayer(L.tileLayer(
    'https://webrd0{s}.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}', amapOpt)),
  '纯卫星': gcjLayer(L.tileLayer('https://webst0{s}.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}', amapOpt)),
  'OpenStreetMap': L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png',
    { maxZoom: 19, attribution: '© OpenStreetMap' }),
};
const map = L.map('map', { zoomControl: true, attributionControl: true }).setView([30.9112, 121.9294], 15);
let base = baseLayers['卫星 + 路网'].addTo(map);
L.control.layers(baseLayers, null, { position: 'topright' }).addTo(map);
const proj = (lat, lon) => (base.gcj ? wgs2gcj(lat, lon) : [lat, lon]);
const unproj = (ll) => (base.gcj ? gcj2wgs(ll.lat, ll.lng) : [ll.lat, ll.lng]).map((v) => +v.toFixed(7));
let routeWgs = [], trackWgs = [], follow = true, centered = false, routeRadius = 3, routeVer = -1, routeClosed = null;
const cvRenderer = L.canvas({ padding: 0.5 });
const routeLine = L.polyline([], { color: '#4a90d9', weight: 3, opacity: 0.85 }).addTo(map);
const wpLayer = L.layerGroup().addTo(map);
const trackLine = L.polyline([], { color: '#ff9f1c', weight: 3 }).addTo(map);
const wpCircle = L.circle([0, 0], { radius: 3, color: '#e040fb', weight: 1.5, fillColor: '#e040fb', fillOpacity: 0.25 });
const wpLink = L.polyline([], { color: '#e040fb', weight: 2.5, dashArray: '6 5', opacity: 0.95 });
const wpNext = L.circleMarker([0, 0], { radius: 4, color: '#fff', weight: 1.5, fillColor: '#e040fb', fillOpacity: 1 });
const accCircle = L.circle([0, 0], { radius: 1, color: '#3fb950', weight: 1, fillOpacity: 0.12 });
const robotDot = L.circleMarker([0, 0], { radius: 7, color: '#fff', weight: 2, fillColor: '#3fb950', fillOpacity: 1 });
let wpMarkers = [], wpDone = null;
function wpStyle(i) {
  const done = wpDone && wpDone(i);
  return { radius: 3.5, weight: 1, color: '#fff', fillColor: done ? '#7d8590' : '#4a90d9', fillOpacity: done ? 0.6 : 1 };
}
function redrawGeo() {
  routeLine.setLatLngs(routeWgs.map(([a, b]) => proj(a, b)));
  trackLine.setLatLngs(trackWgs.map(([a, b]) => proj(a, b)));
  wpLayer.clearLayers();
  // a closed loop repeats the first point at the end; draw it once
  const n = routeWgs.length > 1 && (routeClosed || routeClosed === null)
    && distM(routeWgs[0], routeWgs.at(-1)) < 0.5 ? routeWgs.length - 1 : routeWgs.length;
  wpMarkers = routeWgs.slice(0, n).map(([a, b], i) =>
    L.circleMarker(proj(a, b), { renderer: cvRenderer, ...wpStyle(i) }).bindTooltip(`路径点 #${i}`).addTo(wpLayer));
  if (typeof drawPreview === 'function') drawPreview();
}
function loadRoute() {
  fetch('/api/route').then((r) => (r.ok ? r.json() : null)).then((g) => {
    if (!g || !g.points) return;
    routeWgs = g.points;
    routeRadius = g.radius || 3;
    routeClosed = g.closed ?? null;
    redrawGeo();
    if (!centered) map.fitBounds(routeLine.getBounds(), { padding: [10, 10] });
  });
}
map.on('baselayerchange', (e) => { base = e.layer; redrawGeo(); if (S.state) updateMap(S.state); });
map.on('dragstart', () => { follow = false; $('follow').classList.remove('on'); });
$('follow').onclick = () => { follow = !follow; $('follow').classList.toggle('on', follow); if (S.state) updateMap(S.state); };
fetch('/api/track').then((r) => r.json()).then((t) => { trackWgs = t; redrawGeo(); });

// metres between two WGS-84 points (local flat earth, fine for < 10 km)
function distM(a, b) {
  const k = 111320, c = Math.cos(a[0] * Math.PI / 180);
  return Math.hypot((b[0] - a[0]) * k, (b[1] - a[1]) * k * c);
}
// without a running follower: next waypoint = end of the closest route segment (estimate)
function estimateNext(pos) {
  if (routeWgs.length < 2) return null;
  const k = 111320, c = Math.cos(pos[0] * Math.PI / 180);
  const xy = ([la, lo]) => [(lo - pos[1]) * k * c, (la - pos[0]) * k];
  let best = null;
  for (let i = 0; i + 1 < routeWgs.length; i++) {
    const [ax, ay] = xy(routeWgs[i]), [bx, by] = xy(routeWgs[i + 1]);
    const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy || 1e-9;
    const t = Math.max(0, Math.min(1, -(ax * dx + ay * dy) / l2));
    const d = Math.hypot(ax + t * dx, ay + t * dy);
    if (!best || d < best.d) best = { d, i };
  }
  let i = best.i + 1;
  if (distM(pos, routeWgs[i]) <= routeRadius && i + 1 < routeWgs.length) i += 1;
  if (i === routeWgs.length - 1 && wpMarkers.length === routeWgs.length - 1) i = 0;
  return { idx: i, lat: routeWgs[i][0], lon: routeWgs[i][1], radius: routeRadius, est: true };
}

function updateWaypoint(st, pos) {
  const nav = st.nav;
  let wp = null;
  if (nav && nav.wp_lat != null) {
    wp = { idx: nav.wp_idx, lat: nav.wp_lat, lon: nav.wp_lon, radius: nav.wp_radius_m, est: false };
    const n = wpMarkers.length, first = nav.wp_first;
    let span = nav.wp_idx < 0 ? n : (nav.wp_idx - first + n) % n;
    if (span === 0 && nav.progress_m > nav.route_m / 2) span = n;   // back at the start of a loop
    const key = `${first}:${span}`;
    if (first != null && wpDone?.key !== key) {
      wpDone = (i) => ((i - first + n) % n) < span;
      wpDone.key = key;
      wpMarkers.forEach((m, i) => m.setStyle(wpStyle(i)));
    }
  } else {
    if (wpDone) { wpDone = null; wpMarkers.forEach((m, i) => m.setStyle(wpStyle(i))); }
    if (pos) wp = estimateNext(pos);
  }
  if (!wp || !pos) {
    [wpCircle, wpLink, wpNext].forEach((l) => l.remove());
    return '';
  }
  const p = proj(wp.lat, wp.lon);
  wpCircle.setLatLng(p).setRadius(wp.radius).addTo(map);
  wpLink.setLatLngs([proj(pos[0], pos[1]), p]).addTo(map);
  wpNext.setLatLng(p).addTo(map);
  const d = distM(pos, [wp.lat, wp.lon]);
  const label = `下一点 ${wp.idx < 0 ? '终点' : '#' + wp.idx} · ${d.toFixed(1)} m · 半径 ${wp.radius} m`
    + (wp.est ? (st.nav ? `（导航 ${st.nav.state}，按最近路段估计）` : '（导航未运行，按最近路段估计）') : '');
  wpNext.bindTooltip(label);
  return label;
}

function updateMap(st) {
  if (routeVer !== st.route_ver) { routeVer = st.route_ver; loadRoute(); }
  const f = st.fix;
  if (!f) { $('gpsinfo').textContent = '无 GPS 数据'; updateWaypoint(st, null); return; }
  if (!f.ok) { $('gpsinfo').textContent = '未定位（室内或刚上电，需要开阔天空）'; updateWaypoint(st, null); return; }
  const p = proj(f.lat, f.lon);
  robotDot.setLatLng(p).addTo(map);
  accCircle.setLatLng(p).setRadius(f.acc || 3).addTo(map);
  robotDot.bringToFront();
  const last = trackWgs[trackWgs.length - 1];
  if (st.track_len > trackWgs.length && (!last || last[0] !== f.lat || last[1] !== f.lon)) {
    trackWgs.push([f.lat, f.lon]);
    trackLine.addLatLng(p);
  }
  // the follower's fused position is what it steers from; fall back to the raw fix
  const pos = st.nav && st.nav.lat != null ? [st.nav.lat, st.nav.lon] : [f.lat, f.lon];
  const wpText = updateWaypoint(st, pos);
  $('gpsinfo').textContent = `${f.lat.toFixed(6)}, ${f.lon.toFixed(6)} ±${f.acc ?? '?'} m`
    + (wpText ? ` · ${wpText}` : '');
  if (follow) {
    if (!centered) { map.setView(p, 18); centered = true; } else map.panTo(p, { animate: false });
  }
}

// ---------------------------------------------------------------- lidar top view
const cv = $('lidarcv'), ctx = cv.getContext('2d');
let lidarPts = [];   // x, y, z in cm (base_link), highest point per grid cell
// height colour map: blue (low, curbs) -> cyan -> green -> yellow -> red (>= Z_HI, head height)
const Z_LO = 0.05, Z_HI = 1.5, N_COL = 24;
const Z_COLORS = Array.from({ length: N_COL }, (_, i) => `hsl(${240 - 240 * i / (N_COL - 1)},95%,58%)`);
const zBin = (z) => Math.max(0, Math.min(N_COL - 1, Math.floor((z - Z_LO) / (Z_HI - Z_LO) * N_COL)));
function drawLidar() {
  const st = S.state, dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth * dpr; if (cv.width !== w) { cv.width = w; cv.height = w; }
  const R = +$('lrange').value, s = w / (2 * R), c = w / 2;
  ctx.clearRect(0, 0, w, w);
  ctx.strokeStyle = '#263040'; ctx.fillStyle = '#5c6b7d'; ctx.font = `${11 * dpr}px sans-serif`; ctx.lineWidth = dpr;
  for (let r = 1; r <= R; r++) {
    ctx.beginPath(); ctx.arc(c, c, r * s, 0, 2 * Math.PI); ctx.stroke(); ctx.fillText(`${r}m`, c + 3, c - r * s + 12 * dpr);
  }
  // safety stop box (base_link: 0<x<0.45, |y|<0.2)
  const sf = st && st.safety;
  ctx.fillStyle = sf && sf.blocked ? 'rgba(248,81,73,.35)' : 'rgba(63,185,80,.18)';
  ctx.fillRect(c - 0.2 * s, c - 0.45 * s, 0.4 * s, 0.45 * s);
  const ps = Math.max(1.5 * dpr, 0.06 * s);
  const bins = Array.from({ length: N_COL }, () => []);
  for (let i = 0; i + 2 < lidarPts.length; i += 3) {
    const x = lidarPts[i] / 100, y = lidarPts[i + 1] / 100;
    if (Math.abs(x) > R || Math.abs(y) > R) continue;
    bins[zBin(lidarPts[i + 2] / 100)].push(c - y * s - ps / 2, c - x * s - ps / 2);
  }
  bins.forEach((b, k) => {   // low first, so high obstacles stay on top
    ctx.fillStyle = Z_COLORS[k];
    for (let j = 0; j < b.length; j += 2) ctx.fillRect(b[j], b[j + 1], ps, ps);
  });
  // height legend (bottom-left)
  const lh = 0.32 * w, lw = 8 * dpr, lx = 6 * dpr, ly = w - 8 * dpr - lh;
  Z_COLORS.forEach((col, k) => { ctx.fillStyle = col; ctx.fillRect(lx, ly + lh * (1 - (k + 1) / N_COL), lw, lh / N_COL + 1); });
  ctx.fillStyle = '#9fb0c4'; ctx.font = `${10 * dpr}px sans-serif`;
  ctx.fillText(`${Z_HI} m`, lx + lw + 3 * dpr, ly + 8 * dpr);
  ctx.fillText(`${((Z_LO + Z_HI) / 2).toFixed(1)} m`, lx + lw + 3 * dpr, ly + lh / 2 + 4 * dpr);
  ctx.fillText(`${Z_LO} m`, lx + lw + 3 * dpr, ly + lh);
  // CityWalker local plan (base_link, metres), magenta
  const cwp = st && st.nav && st.nav.cw && st.nav.cw.path;
  if (cwp && cwp.length) {
    ctx.strokeStyle = '#ff4fd8'; ctx.fillStyle = '#ff4fd8'; ctx.lineWidth = 2.5 * dpr;
    ctx.beginPath(); ctx.moveTo(c, c);
    cwp.forEach(([x, y]) => ctx.lineTo(c - y * s, c - x * s));
    ctx.stroke();
    cwp.forEach(([x, y]) => { ctx.beginPath(); ctx.arc(c - y * s, c - x * s, 3.5 * dpr, 0, 2 * Math.PI); ctx.fill(); });
    ctx.lineWidth = dpr;
  }
  // robot footprint (S1 ~0.32 x 0.24 m), forward = up
  ctx.fillStyle = '#4a90d9'; ctx.fillRect(c - 0.12 * s, c - 0.16 * s, 0.24 * s, 0.32 * s);
  ctx.beginPath(); ctx.moveTo(c, c - 0.28 * s); ctx.lineTo(c - 0.07 * s, c - 0.17 * s); ctx.lineTo(c + 0.07 * s, c - 0.17 * s);
  ctx.fill();
}
$('lrange').onchange = drawLidar;
window.addEventListener('resize', drawLidar);

// ---------------------------------------------------------------- status tables
const TOPICS = [
  ['camera', '相机', 8], ['lidar', '激光雷达', 10], ['lidar_imu', '雷达 IMU', 200], ['odom', '底盘里程计', 20],
  ['chassis_imu', '底盘 IMU', 20], ['battery', '电池', 1], ['gps', 'GPS', 1], ['safety', '安全节点', 5],
  ['wheel_cmd', '轮速指令', 20, true], ['vln_cmd', 'VLN/遥控指令', 0, true], ['nav', 'GPS 导航', 2, true],
];
function topicClass(t, exp, optional) {
  if (!t || t.age === null) return optional ? '' : 'bad';
  if (t.age > Math.max(2, 3 / Math.max(exp, 0.1))) return optional ? '' : 'bad';
  if (exp && t.hz < exp * 0.5) return 'warn';
  return 'ok';
}
function renderTopics(st) {
  $('topics').innerHTML = TOPICS.map(([k, name, exp, opt]) => {
    const t = st.topics[k], cls = topicClass(t, exp, opt);
    const val = t && t.age !== null ? `${t.hz} Hz` : (opt ? '空闲' : '无数据');
    const age = t && t.age !== null ? `${t.age}s` : '';
    return `<tr><td><span class="dot ${cls}"></span>${name}</td><td>${val}</td><td class="sub">${age}</td></tr>`;
  }).join('');
}
const fmt = (v, d = 2) => (v === null || v === undefined ? '--' : (+v).toFixed(d));
function renderInfo(st) {
  const o = st.odom || {}, i = st.imu || {}, b = st.battery || {}, s = st.sys || {}, f = st.fix || {}, sf = st.safety || {};
  const rows = [
    ['里程计', `x ${fmt(o.x)} y ${fmt(o.y)} m · 航向 ${fmt(o.yaw, 1)}°`],
    ['速度', `vx ${fmt(o.vx)} vy ${fmt(o.vy)} m/s · wz ${fmt(o.wz)} rad/s`],
    ['姿态', `横滚 ${fmt(i.roll, 1)}° 俯仰 ${fmt(i.pitch, 1)}°`],
    ['GPS', f.ok ? `${fmt(f.lat, 6)}, ${fmt(f.lon, 6)} · 海拔 ${fmt(f.alt, 1)} m · ±${f.acc ?? '?'} m` : `未定位 (status ${f.status ?? '--'})`],
    ['电池', `${b.pct ?? '--'}% · ${fmt(b.v)} V · ${fmt(b.a)} A · ${fmt(b.temp, 1)}°C`],
    ['轮速输出', st.wheel_cmd ? st.wheel_cmd.join(' / ') : '--'],
    ['安全', `${sf.blocked ? '⚠ 前方障碍' : '前方通畅'} · 雷达${sf.lidar_ok ? '正常' : '超时'} · 限速 ${sf.max_v ?? '--'} m/s`],
    ['S1 连接', st.s1_connected === null || st.s1_connected === undefined ? '--' : (st.s1_connected ? '已连接' : '⚠ 断开')],
    ['车载电脑', `CPU ${s.cpu ?? '--'}% · 内存 ${s.mem ?? '--'}% · ${s.temp ?? '--'}°C · 负载 ${s.load ?? '--'}`],
    ['网络', `${s.wifi ?? '--'} (${s.wifi_signal ?? '--'}%) · 延迟 ${S.rtt ?? '--'} ms · 在线 ${st.clients}`],
  ];
  $('info').innerHTML = rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join('');
}
function renderHeader(st) {
  const b = st.battery, s = st.sys || {}, sf = st.safety;
  $('batt').textContent = `🔋 ${b && b.pct !== null ? b.pct + '%' : '--'}`;
  $('batt').className = 'pill ' + (!b ? '' : b.pct > 30 ? 'ok' : b.pct > 15 ? 'warn' : 'bad');
  $('wifi').textContent = `📶 ${s.wifi_signal ?? '--'}%`;
  $('wifi').className = 'pill ' + (s.wifi_signal > 50 ? 'ok' : s.wifi_signal > 25 ? 'warn' : 'bad');
  if (!sf) { $('safety').textContent = '安全节点离线'; $('safety').className = 'pill bad'; }
  else if (sf.estop) { $('safety').textContent = '急停中'; $('safety').className = 'pill bad'; }
  else if (sf.blocked) { $('safety').textContent = '前方障碍'; $('safety').className = 'pill warn'; }
  else { $('safety').textContent = '安全'; $('safety').className = 'pill ok'; }
  S.estop = st.estop;
  $('estop').classList.toggle('active', st.estop);
  $('estop').textContent = st.estop ? '解除急停' : '急停';
  setControlsEnabled();
  const cam = st.topics.camera;
  $('camnote').textContent = cam && cam.age !== null ? `源 ${cam.hz} Hz · 延迟 ${cam.age}s` : '无相机数据';
}

// ---------------------------------------------------------------- driver processes (reconnect)
const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => `&#${c.charCodeAt(0)};`);
function renderModules(st) {
  const mods = st.modules || {};
  const names = Object.keys(mods);
  $('restartall').style.display = st.managed ? '' : 'none';
  const html = !names.length ? '<tr><td>驱动不由网页管理</td></tr>' : names.map((n) => {
    const m = mods[n];
    const cls = m.busy ? 'warn' : m.running ? 'ok' : m.wanted ? 'bad' : '';
    const txt = m.busy ? '重启中…' : m.running ? '运行中' : m.wanted ? '已退出，等待重启' : '未启动';
    const extra = m.restarts ? ` · 重启 ${m.restarts} 次` : '';
    const btn = m.running || m.wanted ? '重连' : '启动';
    return `<tr><td><span class="dot ${cls}"></span>${esc(m.label)}</td><td>${txt}${extra}</td>` +
      `<td><button class="small" data-mod="${esc(n)}" ${m.busy ? 'disabled' : ''}>${btn}</button></td></tr>`;
  }).join('');
  // only touch the DOM on change, otherwise a click can land on a replaced button
  if (html !== renderModules.last) { $('modules').innerHTML = html; renderModules.last = html; }
}
$('modules').onclick = async (e) => {
  const n = e.target.dataset && e.target.dataset.mod;
  if (!n) return;
  e.target.disabled = true;
  const m = (S.state.modules || {})[n] || {};
  await post('/api/module', { name: n, action: m.running || m.wanted ? 'restart' : 'start' });
};
$('restartall').onclick = async () => {
  if (!confirm('重启全部驱动？（底盘会短暂停止响应）')) return;
  disarm();
  await post('/api/module', { name: 'all', action: 'restart' });
};

// ---------------------------------------------------------------- VLN task
const VLN_STATE = {
  idle: ['空闲 · 等待指令', ''], waiting_image: ['等待图像', 'warn'], waiting_depth: ['等待深度图', 'warn'],
  requesting: ['请求服务器中', 'run'], executing: ['执行中', 'run'], done: ['已完成', 'ok'],
  stopped: ['已停止', ''], error: ['出错', 'bad'],
};
function renderVln(st) {
  const v = st.vln, sv = st.server || {};
  let label, cls;
  if (st.vln_pending) [label, cls] = ['启动客户端中…', 'warn'];
  else if (!v) [label, cls] = ['客户端未运行', ''];
  else [label, cls] = VLN_STATE[v.state] || [v.state, ''];
  $('vlnstate').textContent = label; $('vlnstate').className = 'pill ' + cls;
  $('vlnpill').textContent = 'VLN ' + label; $('vlnpill').className = 'pill ' + cls;
  const r = (v && v.reply) || {};
  let act = '--';
  if (r.discrete_action) act = '离散动作 ' + r.discrete_action.map((a) => ({ 0: '停', 1: '前', 2: '左', 3: '右', 5: '低头' }[a] ?? a)).join(' ');
  else if (r.traj_n) act = `轨迹 ${r.traj_n} 点 → 终点 (${r.traj_end ? r.traj_end.join(', ') : '?'}) m`;
  const rows = [
    ['GPU 服务器', sv.ok === null || sv.ok === undefined ? '检测中…' : sv.ok
      ? (sv.info && sv.info.ok === false ? '<span class="dot warn"></span>在线 · 模型加载中'
        : '<span class="dot ok"></span>在线 · 模型就绪') + ` · ${Math.round((sv.rtt || 0) * 1000)} ms`
      : `<span class="dot bad"></span>离线 <span class="sub">${esc(sv.error || '')}</span>`],
    ['当前指令', v && v.instruction ? esc(v.instruction) : (st.vln_pending ? esc(st.vln_pending) : '--')],
    ['步数 / 用时', v ? `${v.idx} 步 · ${v.elapsed ?? '--'} s` : '--'],
    ['服务器回复', act + (r.latency_s !== undefined ? ` · 推理 ${fmt(r.latency_s)} s` : '') +
      (v && v.rtt ? ` · 往返 ${fmt(v.rtt)} s` : '')],
  ];
  if (v && v.error) rows.push(['错误', `<span style="color:#f85149">${esc(v.error)}</span>`]);
  if (v && v.msg) rows.push(['日志', esc(v.msg)]);
  $('vlninfo').innerHTML = rows.map(([k, val]) => `<tr><td>${k}</td><td>${val}</td></tr>`).join('');
}
$('vlnform').onsubmit = async (e) => {
  e.preventDefault();
  const text = $('vlntext').value.trim();
  if (!text) return;
  if (S.estop) { alert('先解除急停'); return; }
  if (!confirm(`机器人将自主移动执行：\n${text}\n\n确认周围安全？`)) return;
  disarm();
  const r = await post('/api/vln', { instruction: text });
  if (r.error) alert('发送失败：' + r.error);
};
$('vlnstop').onclick = () => post('/api/vln/stop');

// ---------------------------------------------------------------- GPS navigation task
const NT = { pick: null, start: null, end: null, points: [], preview: null, routes: [], amap: false, timer: null };
const NAV_SLIDERS = ['navradius', 'navv', 'navspacing', 'navtol'];
const pvLayer = L.layerGroup().addTo(map);
const pickLayer = L.layerGroup().addTo(map);
function navReq() {
  return {
    mode: $('navmode').value, base: $('navbase').value, start: NT.start, end: NT.end, points: NT.points,
    reverse: $('navrev').checked, loop: $('navloop').checked, radius: +$('navradius').value,
    max_v: +$('navv').value, spacing: +$('navspacing').value, tol: +$('navtol').value, planner: $('navplanner').value,
  };
}
function drawPreview() {
  pvLayer.clearLayers(); pickLayer.clearLayers();
  const pv = NT.preview;
  if (pv && pv.points.length) {
    const ll = pv.points.map(([a, b]) => proj(a, b));
    L.polyline(ll, { color: '#ff9f1c', weight: 4, opacity: 0.9 }).addTo(pvLayer);
    const n = pv.closed && ll.length > 1 ? ll.length - 1 : ll.length;
    for (let i = 0; i < n; i++) {
      L.circle(ll[i], { renderer: cvRenderer, radius: pv.radius, color: '#ff9f1c', weight: 1, fillColor: '#ff9f1c',
                        fillOpacity: 0.18, interactive: false }).addTo(pvLayer);
      const m = L.circleMarker(ll[i], { renderer: cvRenderer, radius: 4, color: '#fff', weight: 1, fillColor: '#ff9f1c',
                                        fillOpacity: 1 }).addTo(pvLayer);
      if (n <= 80) m.bindTooltip(String(i), { permanent: true, direction: 'right', className: 'wplabel', offset: [4, 0] });
      else m.bindTooltip(`预览点 #${i}`);
    }
  }
  const mark = (p, color, txt) => p && L.circleMarker(proj(p[0], p[1]), { radius: 8, color: '#fff', weight: 2,
    fillColor: color, fillOpacity: 1 }).bindTooltip(txt).addTo(pickLayer);
  mark(NT.start, '#3fb950', '起点');
  mark(NT.end, '#f85149', '终点');
  NT.points.forEach((p, i) => mark(p, '#a371f7', `途经点 ${i + 1}`));
}
function renderPickInfo() {
  const f = (p, dflt) => (p ? `${p[0].toFixed(6)}, ${p[1].toFixed(6)}` : dflt);
  const mode = $('navmode').value;
  const dEnd = mode === 'route' ? ($('navloop').checked ? '回到起点' : '路线终点') : '未选';
  $('navpts').textContent = `起点：${f(NT.start, mode === 'amap' ? '当前位置' : mode === 'route' ? '路线起点' : '未选')} · `
    + `终点：${f(NT.end, dEnd)}` + (mode === 'manual' ? ` · 途经点 ${NT.points.length} 个` : '');
  document.querySelectorAll('.navpick [data-pick]').forEach((b) => b.classList.toggle('on', NT.pick === b.dataset.pick));
  $('map').classList.toggle('picking', !!NT.pick);
}
function navFormChanged() {
  const mode = $('navmode').value;
  $('navbasel').style.display = mode === 'route' ? '' : 'none';
  $('navrevl').style.display = mode === 'route' ? '' : 'none';
  $('pickpts').style.display = mode === 'manual' ? '' : 'none';
  navSliderLabels();
  renderPickInfo();
  clearTimeout(NT.timer);
  if (mode === 'amap') { NT.preview = null; drawPreview(); showPreviewInfo(null, '高德规划需点“预览”'); return; }
  NT.timer = setTimeout(previewTask, 300);
}
let navInfoNote = null;
function showPreviewInfo(pv, err) {
  navInfoNote = err ? `<span style="color:#f85149">${esc(err)}</span>` : pv
    ? `预览：${pv.n} 个路径点 · ${(pv.length_m / 1000).toFixed(2)} km · ${pv.closed ? '闭环' : '单程'} · `
      + `约 ${Math.round(pv.eta_s / 60)} 分钟 @ ${pv.max_v} m/s`
      + (pv.warnings && pv.warnings.length ? ` · <span style="color:#d29922">⚠ ${esc(pv.warnings.join('；'))}</span>` : '')
    : null;
  if (S.state) renderNav(S.state);
}
async function previewTask() {
  const req = navReq();
  if (req.mode === 'manual' && [req.start, ...req.points, req.end].filter(Boolean).length < 2) {
    NT.preview = null; drawPreview(); showPreviewInfo(null, '手动模式：在地图上选起点、途经点和终点'); return null;
  }
  const r = await post('/api/nav/preview', { task: req });
  if (r.error || !r.ok) { NT.preview = null; drawPreview(); showPreviewInfo(null, r.error || '预览失败'); return null; }
  NT.preview = r; drawPreview(); showPreviewInfo(r);
  return r;
}
map.on('click', (e) => {
  if (!NT.pick) return;
  const p = unproj(e.latlng);
  if (NT.pick === 'points') NT.points.push(p);
  else { NT[NT.pick] = p; NT.pick = null; }
  navFormChanged();
});
document.querySelectorAll('.navpick [data-pick]').forEach((b) => {
  b.onclick = () => { NT.pick = NT.pick === b.dataset.pick ? null : b.dataset.pick; follow = false;
    $('follow').classList.remove('on'); renderPickInfo(); };
});
$('herestart').onclick = () => {
  const f = S.state && S.state.fix;
  if (!f || !f.ok) { alert('当前没有 GPS 定位'); return; }
  NT.start = [f.lat, f.lon]; navFormChanged();
};
$('navundo').onclick = () => {
  if (NT.points.length && $('navmode').value === 'manual') NT.points.pop();
  else if (NT.end) NT.end = null; else NT.start = null;
  navFormChanged();
};
$('navclear').onclick = () => { NT.start = NT.end = null; NT.points = []; NT.pick = null; navFormChanged(); };
function navSliderLabels() { NAV_SLIDERS.forEach((id) => { $(id + 'v').textContent = $(id).value; }); }
['navmode', 'navbase', 'navrev', 'navloop', 'navplanner', ...NAV_SLIDERS].forEach((id) => {
  $(id).addEventListener(NAV_SLIDERS.includes(id) ? 'input' : 'change', navFormChanged);
});
$('navpreview').onclick = async () => {
  const r = await previewTask();
  if (r && r.points.length) map.fitBounds(L.latLngBounds(r.points.map(([a, b]) => proj(a, b))), { padding: [20, 20] });
};
$('navgo').onclick = async () => {
  if (S.estop) { alert('先解除急停'); return; }
  const pv = await previewTask();
  if (!pv) return;
  if (!confirm(`启动 GPS 导航：${pv.n} 个路径点，${(pv.length_m / 1000).toFixed(2)} km，限速 ${pv.max_v} m/s。\n`
    + '机器人将自主移动，确认周围安全？')) return;
  disarm();
  const r = await post('/api/nav/start', { task: navReq() });
  if (r.error) { alert('启动失败：' + r.error); return; }
  NT.preview = null; NT.pick = null; drawPreview(); showPreviewInfo(null); renderPickInfo();
};
$('navresume').onclick = async () => {
  if (S.estop) { alert('先解除急停'); return; }
  if (!confirm('继续当前 GPS 导航任务？机器人将自主移动。')) return;
  disarm();
  const r = await post('/api/nav/start', {});
  if (r.error) alert('继续失败：' + r.error);
};
$('navpause').onclick = () => post('/api/nav/pause');
$('navcancel').onclick = () => { if (confirm('结束当前导航任务？')) post('/api/nav/cancel'); };
fetch('/api/nav/routes').then((r) => r.json()).then((d) => {
  NT.routes = d.routes || []; NT.amap = d.amap;
  $('navbase').innerHTML = NT.routes.map((n) => `<option>${esc(n)}</option>`).join('');
  $('navmode').querySelector('option[value=amap]').textContent = '高德步行规划' + (d.amap ? '' : '（未配置 Key）');
  const q = d.task && d.task.req;
  if (q) {  // restore the form from the saved task
    $('navmode').value = q.mode || 'route';
    if (q.base && NT.routes.includes(q.base)) $('navbase').value = q.base;
    NT.start = q.start || null; NT.end = q.end || null; NT.points = q.points || [];
    $('navrev').checked = !!q.reverse; $('navloop').checked = !!q.loop;
    $('navradius').value = q.radius ?? 3; $('navv').value = q.max_v ?? 0.3; $('navspacing').value = q.spacing ?? 25;
    $('navtol').value = q.tol ?? 1; $('navplanner').value = q.planner || 'gps';
  }
  const mode = $('navmode').value;
  $('navbasel').style.display = mode === 'route' ? '' : 'none';
  $('navrevl').style.display = mode === 'route' ? '' : 'none';
  $('pickpts').style.display = mode === 'manual' ? '' : 'none';
  navSliderLabels();
  renderPickInfo(); drawPreview();
}).catch(() => {});

const NAV_STATE = {
  WAIT_FIX: ['等待 GPS 定位', 'warn'], ALIGN: ['对准航向', 'run'], FOLLOW: ['导航中', 'run'],
  HOLD: ['GPS 丢失，原地等待', 'warn'], DONE: ['已到达终点', 'ok'], FAULT: ['故障停止', 'bad'],
};
function renderNav(st) {
  const nav = st.nav, task = st.nav_task, mod = (st.modules || {}).nav || {};
  let label, cls;
  if (st.nav_pending) [label, cls] = ['启动中…', 'warn'];
  else if (!nav) [label, cls] = [task ? (mod.wanted ? '导航节点启动中' : '未运行（可继续）') : '无任务', ''];
  else if (nav.state === 'DONE' || nav.state === 'FAULT') [label, cls] = NAV_STATE[nav.state];
  else if (!nav.enabled) [label, cls] = ['已暂停', 'warn'];
  else [label, cls] = NAV_STATE[nav.state] || [nav.state, ''];
  $('navstate').textContent = label; $('navstate').className = 'pill ' + cls;
  const moving = !!(nav && nav.enabled) || st.nav_pending;
  $('navpause').disabled = !moving;
  $('navresume').disabled = !task || moving || st.estop;
  $('navgo').disabled = !!st.estop;
  $('navcancel').disabled = !(mod.running || mod.wanted || st.nav_pending);
  $('navresume').textContent = nav && nav.state === 'DONE' ? '重新开始' : '继续';
  const MODE = { route: '沿路线', manual: '手动点选', amap: '高德规划' };
  const rows = [];
  if (task) {
    rows.push(['任务', `${MODE[task.mode] || task.mode}${task.base ? ' · ' + esc(task.base) : ''}${task.reverse ? ' · 反向' : ''}`
      + ` · ${task.n} 点 · ${(task.length_m / 1000).toFixed(2)} km · ${task.closed ? '闭环' : '单程'}`
      + ` · 半径 ${task.radius} m · 限速 ${task.max_v} m/s · ${task.planner === 'citywalker' ? 'CityWalker' : 'GPS 追踪'}`]);
  }
  if (nav) {
    const pct = nav.route_m ? Math.min(100, (100 * nav.progress_m) / nav.route_m).toFixed(0) : '--';
    rows.push(['进度', `${fmt(nav.progress_m, 0)} / ${fmt(nav.route_m, 0)} m（${pct}%）· 偏离 ${fmt(nav.xte_m, 1)} m`]);
    if (nav.wp_idx != null) rows.push(['下一点', `${nav.wp_idx < 0 ? '终点' : '#' + nav.wp_idx} / ${nav.wp_n} · ${fmt(nav.wp_dist_m, 1)} m`]);
    const cw = nav.cw;
    if (cw) {
      const srv = cw.ok === null ? '未请求' : cw.ok ? `正常 ${cw.rtt_ms ?? '--'} ms（推理 ${fmt(cw.infer_ms, 0)} ms）` : `<span class="bad">错误</span> ${esc(cw.err)}`;
      rows.push(['CityWalker', `${cw.active ? '<b>规划中</b>' : 'GPS 追踪（回退）'} · ${srv} · 历史帧 ${cw.frames}/5`
        + (cw.arrive != null ? ` · 到达概率 ${fmt(cw.arrive, 2)}` : '') + ` · ${cw.n} 次/${cw.n_err} 错`]);
    }
    if (nav.reason) rows.push(['原因', esc(nav.reason)]);
  }
  if (st.nav_msg) rows.push(['消息', esc(st.nav_msg)]);
  if (navInfoNote) rows.push(['预览', navInfoNote]);
  const html = rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join('');
  if (html !== renderNav.last) { $('navinfo').innerHTML = html; renderNav.last = html; }
}

// ---------------------------------------------------------------- SSE
let es = null, lastMsg = 0;
function connect() {
  if (es) es.close();
  es = new EventSource(`/api/events?period=0.5&lidar=${$('lidaron').checked ? 1 : 0}`);
  es.onmessage = (ev) => {
    const st = JSON.parse(ev.data);
    S.state = st; lastMsg = Date.now();
    if (st.lidar3) lidarPts = st.lidar3;
    renderHeader(st); renderTopics(st); renderInfo(st); renderModules(st); renderVln(st); renderNav(st); updateMap(st); drawLidar();
  };
  es.onerror = () => { $('conn').textContent = '重连中'; $('conn').className = 'pill bad'; };
}
setInterval(() => {
  const ok = Date.now() - lastMsg < 2500;
  $('conn').textContent = ok ? '已连接' : '断开';
  $('conn').className = 'pill ' + (ok ? 'ok' : 'bad');
  if (!ok && S.armed) disarm('连接断开，已停止遥控');
}, 1000);
$('lidaron').onchange = () => { if (!$('lidaron').checked) { lidarPts = []; drawLidar(); } connect(); };
connect();

// ---------------------------------------------------------------- camera
const CAMQ = { low: 'w=320&fps=3&q=50', mid: 'w=640&fps=5&q=60', high: 'w=640&fps=10&q=75' };
function setCam() {
  const q = $('camq').value;
  $('cam').src = q === 'off' ? '' : `/stream.mjpg?${CAMQ[q]}&_=${Date.now()}`;
}
$('cam').onerror = () => { if ($('camq').value !== 'off') setTimeout(setCam, 2000); };
$('camq').onchange = setCam;
document.addEventListener('visibilitychange', () => {
  if (document.hidden) { $('cam').src = ''; disarm(); } else setCam();
});
setCam();

// ---------------------------------------------------------------- control
const joy = $('joy'), knob = $('knob');
function setControlsEnabled() {
  const en = S.armed && !S.estop;
  joy.classList.toggle('disabled', !en);
  document.querySelector('.btns').classList.toggle('disabled', !en);
}
function disarm(msg) {
  if (S.armed) { S.armed = false; post('/api/stop'); }
  S.keys.clear(); S.joy = [0, 0]; resetKnob(); setControlsEnabled(); renderArm();
  if (msg) $('cmdinfo').textContent = msg;
}
function renderArm() {
  document.querySelectorAll('.armbtn').forEach((b) => {
    b.classList.toggle('on', S.armed);
    b.textContent = S.armed ? '🎮 遥控：开' : '🎮 遥控：关';
  });
}
function toggleArm() {
  if (S.armed) { disarm('遥控已关闭'); return; }
  if (S.estop) { alert('先解除急停'); return; }
  S.armed = true; renderArm(); setControlsEnabled();
  $('cmdinfo').textContent = '遥控已开启 · 按住移动，松手即停 · 再按 T 关闭';
}
document.querySelectorAll('.armbtn').forEach((b) => { b.onclick = (e) => { e.currentTarget.blur(); toggleArm(); }; });
let estopReleasedAt = 0;
$('estop').onclick = async () => {
  if (S.estop) {
    if (!confirm('确认解除急停？')) return;
    estopReleasedAt = Date.now();
    await post('/api/estop', { on: false });
    estopReleasedAt = Date.now();
  } else {
    // a double/ghost tap right after releasing must not re-engage (the button flips under the finger)
    if (Date.now() - estopReleasedAt < 1000) return;
    disarm('已急停');
    await post('/api/estop', { on: true });
  }
};
['vmax', 'wmax'].forEach((id) => { $(id).oninput = () => { $(id + 'v').textContent = $(id).value; }; });

function resetKnob() { knob.style.left = '55px'; knob.style.top = '55px'; }
let joyId = null;
function joyMove(e) {
  const r = joy.getBoundingClientRect(), R = r.width / 2;
  let dx = e.clientX - r.left - R, dy = e.clientY - r.top - R;
  const d = Math.hypot(dx, dy), lim = R - 30;
  if (d > lim) { dx *= lim / d; dy *= lim / d; }
  knob.style.left = `${R - 30 + dx}px`; knob.style.top = `${R - 30 + dy}px`;
  S.joy = [-dy / lim, -dx / lim]; // forward, turn-left
}
joy.addEventListener('pointerdown', (e) => { joyId = e.pointerId; joy.setPointerCapture(e.pointerId); joyMove(e); });
joy.addEventListener('pointermove', (e) => { if (e.pointerId === joyId) joyMove(e); });
['pointerup', 'pointercancel', 'lostpointercapture'].forEach((ev) =>
  joy.addEventListener(ev, () => { joyId = null; S.joy = [0, 0]; resetKnob(); }));

document.querySelectorAll('.btns button').forEach((b) => {
  const k = b.dataset.k;
  b.addEventListener('pointerdown', (e) => { b.setPointerCapture(e.pointerId); S.keys.add(k); b.classList.add('down'); });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((ev) =>
    b.addEventListener(ev, () => { S.keys.delete(k); b.classList.remove('down'); }));
});
window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' && e.target.type !== 'checkbox' && e.target.type !== 'range') return;
  const k = e.key.toLowerCase();
  if (k === 't' && !e.repeat) { toggleArm(); return; }
  if (k === ' ') { e.preventDefault(); S.keys.clear(); S.joy = [0, 0]; post('/api/stop'); return; }
  if ('wasdqe'.includes(k) && k.length === 1) { S.keys.add(k); e.preventDefault(); }
});
window.addEventListener('keyup', (e) => S.keys.delete(e.key.toLowerCase()));
window.addEventListener('blur', () => { S.keys.clear(); S.joy = [0, 0]; resetKnob(); });

let wasMoving = false;
setInterval(async () => {
  if (!S.armed || S.estop) { wasMoving = false; return; }
  const k = S.keys, vmax = +$('vmax').value, wmax = +$('wmax').value;
  let fwd = (k.has('w') ? 1 : 0) - (k.has('s') ? 1 : 0) + S.joy[0];
  let turn = (k.has('a') ? 1 : 0) - (k.has('d') ? 1 : 0) + S.joy[1];
  let side = (k.has('q') ? 1 : 0) - (k.has('e') ? 1 : 0);
  fwd = Math.max(-1, Math.min(1, fwd)); turn = Math.max(-1, Math.min(1, turn));
  const cmd = { vx: +(fwd * vmax).toFixed(3), vy: +(side * vmax).toFixed(3), wz: +(turn * wmax).toFixed(3) };
  const moving = Math.abs(cmd.vx) > 0.005 || Math.abs(cmd.vy) > 0.005 || Math.abs(cmd.wz) > 0.01;
  if (moving) {
    if (S.sending) return;
    S.sending = true;
    try {
      const r = await post('/api/cmd', cmd);
      $('cmdinfo').textContent = `指令 vx ${cmd.vx} vy ${cmd.vy} wz ${cmd.wz} · 延迟 ${S.rtt} ms` +
        (r.blocked && cmd.vx > 0 ? ' · ⚠ 前方障碍，前进被拦' : '');
    } catch (err) { $('cmdinfo').textContent = '发送失败：' + err; }
    S.sending = false;
  } else if (wasMoving) {
    post('/api/stop').catch(() => {});
  }
  wasMoving = moving;
}, 100);
