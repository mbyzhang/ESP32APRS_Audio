// APRS Chat — mobile-first UI
// All compute that *can* live in the browser (TNC2 parsing, track DB, GPS
// throttling, map rendering) lives here.  The firmware just streams raw
// packets over SSE and accepts TX requests as form posts.
//
// Future: Phase 5 will add WebAudio AudioWorklet hooks against the
// existing /ws_audio WebSocket on port 81 for FM voice monitor + PTT.
// We deliberately keep that surface untouched so it can land later.

(() => {
'use strict';

const $ = (s) => document.querySelector(s);
const feed   = $('#feed');
const status = $('#status');
const meCall = $('#meCall');
const meIp   = $('#meIp');
const toCall = $('#toCall');
const msgIn  = $('#msgInput');
const sendBtn= $('#sendBtn');
const gpsBtn = $('#gpsBtn');

let me = { callsign: '', ssid: 0 };
const seen = new Set();          // dedupe by raw+ts
const trackDb = openTrackDb();   // opened lazily, IndexedDB; future Phase 3

// ---------- TNC2 / APRS parsing (browser-side) -----------------------------

// Parse a TNC2 packet line "SRC>DST,PATH:INFO" into structured form.
// Returns at least {src, dst, path, info, type, raw} on success, plus
// position/message fields when applicable. type ∈ {position,message,status,object,unknown}.
function parseTnc2(raw) {
  if (!raw) return null;
  const colon = raw.indexOf(':');
  const gt    = raw.indexOf('>');
  if (gt < 0 || colon < gt) return null;
  const src    = raw.slice(0, gt);
  const header = raw.slice(gt + 1, colon);
  const info   = raw.slice(colon + 1);
  const parts  = header.split(',');
  const dst    = parts[0];
  const path   = parts.slice(1).join(',');
  const out = { raw, src, dst, path, info, type: 'unknown' };
  if (!info.length) return out;
  const t = info[0];
  if (t === '!' || t === '=' || t === '/' || t === '@') {
    Object.assign(out, parsePositionInfo(info));
    out.type = 'position';
  } else if (t === ':' && info.length >= 11) {
    // Message: ":<addressee:9chars>:<text>{<msgid>}"
    // Some senders use spaces to pad addressee.
    const addressee = info.slice(1, 10).trim().toUpperCase();
    const rest = info.slice(11);
    let text = rest, msgid = null;
    const lc = rest.lastIndexOf('{');
    if (lc >= 0) { text = rest.slice(0, lc); msgid = rest.slice(lc + 1); }
    out.type = 'message';
    out.addressee = addressee;
    out.message = text;
    out.msgid = msgid;
  } else if (t === '>') {
    out.type = 'status';
    out.status = info.slice(1);
  } else if (t === ';') {
    out.type = 'object';
    out.object = info.slice(1, 10).trim();
    Object.assign(out, parsePositionInfo(info.slice(17))); // approx — best-effort
  }
  return out;
}

// Decode the lat/lon body of a position packet.  Handles both the
// uncompressed ddmm.hhN/dddmm.hhW format and the 13-char compressed form.
function parsePositionInfo(info) {
  // Strip leading type char if it is one of !=/@
  if ('!=/@'.indexOf(info[0]) >= 0) info = info.slice(1);
  // Optional 7-char timestamp on @ and /
  if (info.length > 7 && /^[0-9]{6}[zh\/]/.test(info)) info = info.slice(7);
  if (info.length < 19) return {};
  // Compressed?  First char in [!-{] but not a digit.
  const first = info.charCodeAt(0);
  if (info.length >= 13 && first >= 0x21 && first <= 0x7b && !/[0-9]/.test(info[0])) {
    // Compressed: SYM_TABLE LAT(4) LON(4) SYM_CODE CS(2) T(1)
    const symT = info[0];
    const lat = 90  - decodeBase91(info.slice(1, 5))  / 380926;
    const lon = -180 + decodeBase91(info.slice(5, 9))  / 190463;
    const symC = info[9];
    const comment = info.slice(13);
    return { lat, lon, symbol_table: symT, symbol_code: symC, comment };
  }
  // Uncompressed: ddmm.hhN/dddmm.hhW>comment
  const m = info.match(/^(\d{2})(\d{2}\.\d{2})([NS])(.)(\d{3})(\d{2}\.\d{2})([EW])(.)(.*)$/);
  if (!m) return {};
  const lat = (parseInt(m[1],10) + parseFloat(m[2])/60) * (m[3]==='S'?-1:1);
  const lon = (parseInt(m[5],10) + parseFloat(m[6])/60) * (m[7]==='W'?-1:1);
  return { lat, lon, symbol_table: m[4], symbol_code: m[8], comment: m[9] };
}
function decodeBase91(s) {
  let v = 0;
  for (let i = 0; i < s.length; i++) v = v * 91 + (s.charCodeAt(i) - 33);
  return v;
}

// ---------- IndexedDB track store (Phase 3 stub, write-only for now) -------
function openTrackDb() {
  return new Promise((resolve) => {
    if (!('indexedDB' in window)) return resolve(null);
    const r = indexedDB.open('aprs-tracks', 1);
    r.onupgradeneeded = () => {
      const db = r.result;
      if (!db.objectStoreNames.contains('points'))
        db.createObjectStore('points', { keyPath: 'id', autoIncrement: true })
          .createIndex('byCall', 'src');
    };
    r.onsuccess = () => resolve(r.result);
    r.onerror   = () => resolve(null);
  });
}
async function recordTrackPoint(pkt) {
  if (!pkt || pkt.lat == null || pkt.lon == null) return;
  // If the map is open, mirror this point onto it immediately so users
  // see live movement without reopening the overlay.
  pushPointToMap(pkt);
  const db = await trackDb;
  if (!db) return;
  try {
    db.transaction('points', 'readwrite').objectStore('points').add({
      src: pkt.src, ts: pkt.ts, lat: pkt.lat, lon: pkt.lon,
      symbol: (pkt.symbol_table || '') + (pkt.symbol_code || ''),
      comment: pkt.comment || ''
    });
  } catch (_) {}
}

// ---------- Rendering -----------------------------------------------------

function setStatus(text, cls) {
  status.textContent = text;
  status.className = cls || 'dim';
}
function fmtTime(ts) {
  const d = new Date(ts * 1000);
  return d.toLocaleTimeString([], { hour:'2-digit', minute:'2-digit', second:'2-digit' });
}
function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({
    '&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'
  }[c]));
}

function renderPacket(pkt) {
  const key = pkt.ts + '|' + pkt.raw;
  if (seen.has(key)) return;
  seen.add(key);
  if (seen.size > 2000) {
    // bound memory
    const drop = seen.values().next().value;
    seen.delete(drop);
  }

  const parsed = parseTnc2(pkt.raw) || { src:'?', type:'unknown' };
  parsed.ts = pkt.ts;
  parsed.audio = pkt.audio;
  parsed.ch = pkt.ch;
  parsed.dir = pkt.dir || 'rx';

  const wasAtBottom = isAtBottom();

  const el = document.createElement('div');
  el.className = 'msg';
  // Treat anything tagged dir=tx OR matching our own callsign as "me".
  const isMine = parsed.dir === 'tx' || parsed.src === fullCall();
  if (isMine) el.classList.add('me');
  if (parsed.type === 'message' && parsed.addressee === fullCall()) el.classList.add('dm');

  const isMsg = parsed.type === 'message';
  const headRight = parsed.dir === 'tx'
    ? '✓ sent'
    : (pkt.ch === 1 ? 'IS' : (pkt.audio ? `${pkt.audio} dBV` : 'RF'));

  let body = '';
  if (isMsg) {
    body = `<span class="dim">→ ${escapeHtml(parsed.addressee)}:</span> ${escapeHtml(parsed.message || '')}`;
  } else if (parsed.type === 'position') {
    const here = (parsed.lat != null) ? `${parsed.lat.toFixed(5)}, ${parsed.lon.toFixed(5)}` : '';
    body = (parsed.comment || here) ? escapeHtml(parsed.comment || '') : escapeHtml(parsed.info);
    if (parsed.lat != null) {
      body += ` <a target="_blank" rel="noopener" href="https://www.openstreetmap.org/?mlat=${parsed.lat}&mlon=${parsed.lon}#map=14/${parsed.lat}/${parsed.lon}">map</a>`;
    }
  } else if (parsed.type === 'status') {
    body = escapeHtml(parsed.status);
  } else {
    body = escapeHtml(parsed.info || pkt.raw);
  }

  el.innerHTML =
    `<div class="head">
       <span class="src">${escapeHtml(parsed.src)}</span>
       <span class="ts">${fmtTime(pkt.ts)} · ${escapeHtml(headRight)}</span>
     </div>
     <div class="body">${body}</div>
     <div class="meta">
       <span class="pill">${escapeHtml(parsed.type)}</span>
       <span class="pill">${escapeHtml(parsed.path || parsed.dst || '')}</span>
     </div>`;
  feed.appendChild(el);

  // Trim DOM to keep mobile snappy
  while (feed.children.length > 400) feed.removeChild(feed.firstChild);

  if (wasAtBottom) feed.scrollTop = feed.scrollHeight;

  recordTrackPoint(parsed);
}

function isAtBottom() {
  return feed.scrollHeight - feed.scrollTop - feed.clientHeight < 120;
}

// ---------- Network: hydrate + SSE ----------------------------------------

async function loadMe() {
  try {
    const r = await fetch('/api/me');
    if (!r.ok) throw new Error(r.status);
    me = await r.json();
    meCall.textContent = me.ssid ? `${me.callsign}-${me.ssid}` : me.callsign || '(tap to set callsign)';
  } catch (e) {
    meCall.textContent = '(offline)';
  }
}

let radio = {};
async function loadRadio() {
  try {
    const r = await fetch('/api/radio');
    if (!r.ok) return;
    radio = await r.json();
    const f = (radio.freq_rx ?? 0).toFixed(4);
    // Make it obvious when RF is disabled — a chat app silently ignoring all
    // radio traffic because rf_en=false is a frustrating debug session.
    // Show freq + audio level so the user can debug "no RX" at a glance.
    // mvrms = running RMS of demod input in millivolts; if it stays at 0 the
    // audio path from the SA868 to the ADC is dead (cable / module issue).
    const mv = radio.mvrms ?? 0;
    const sq = radio.sql_pin === 0 ? ' · sq' : '';
    $('#meFreq').textContent = radio.rf_en
      ? `${f} MHz · ${mv} mV${sq}`
      : `${f} MHz · RF off`;
    $('#meFreq').classList.toggle('warn', !radio.rf_en);
  } catch (_) {}
}
function fullCall() {
  return me.ssid ? `${me.callsign}-${me.ssid}` : me.callsign;
}

async function hydrate() {
  try {
    const r = await fetch('/api/packets/recent');
    if (!r.ok) return;
    const arr = await r.json();
    // Render oldest first so the live tail appears at the bottom
    arr.reverse().forEach(renderPacket);
    feed.scrollTop = feed.scrollHeight;
  } catch (_) {}
}

let es;
function connectStream() {
  if (es) try { es.close(); } catch (_) {}
  es = new EventSource('/api/packets/stream');
  es.addEventListener('open',  () => setStatus('● live', 'ok'));
  es.addEventListener('error', () => setStatus('reconnecting…', 'warn'));
  es.addEventListener('packet', (ev) => {
    try {
      const pkt = JSON.parse(ev.data);
      renderPacket(pkt);
    } catch (_) {}
  });
}

// ---------- TX -------------------------------------------------------------

async function postForm(url, fields) {
  const body = new URLSearchParams();
  for (const k in fields) if (fields[k] != null) body.set(k, fields[k]);
  const r = await fetch(url, { method:'POST', body });
  let j = null;
  try { j = await r.json(); } catch (_) {}
  return { ok: r.ok && (j?.ok !== false), status: r.status, body: j };
}

async function sendCurrentMessage() {
  const to   = (toCall.value || '').trim().toUpperCase();
  const text = (msgIn.value  || '').trim();
  if (!to || !text) return;
  sendBtn.disabled = true;
  setStatus('sending…', 'warn');
  const res = await postForm('/api/tx/message', { to, text });
  sendBtn.disabled = false;
  if (res.ok) {
    setStatus('sent', 'ok');
    msgIn.value = '';
  } else {
    setStatus('send failed (' + res.status + ')', 'err');
  }
}

// ---------- GPS beacon ----------------------------------------------------

let gpsWatchId = null;
let lastBeacon = { lat:0, lon:0, ts:0 };
const BEACON_MIN_INTERVAL_MS = 60_000;
const BEACON_MIN_MOVE_M = 50;

function haversine(a, b, c, d) {
  const R = 6371000, t = (x) => x * Math.PI / 180;
  const x = Math.sin((c-a)/2 * Math.PI/180);
  const y = Math.sin((d-b)/2 * Math.PI/180);
  const h = x*x + Math.cos(t(a))*Math.cos(t(c))*y*y;
  return 2 * R * Math.asin(Math.sqrt(h));
}

function toggleGps() {
  if (gpsWatchId != null) {
    navigator.geolocation.clearWatch(gpsWatchId);
    gpsWatchId = null;
    gpsBtn.classList.remove('on');
    setStatus('GPS off', 'dim');
    return;
  }
  if (!('geolocation' in navigator)) {
    setStatus('no geolocation', 'err'); return;
  }
  gpsWatchId = navigator.geolocation.watchPosition((pos) => {
    const { latitude:lat, longitude:lon } = pos.coords;
    const now = Date.now();
    const moved = haversine(lastBeacon.lat, lastBeacon.lon, lat, lon);
    if (now - lastBeacon.ts < BEACON_MIN_INTERVAL_MS && moved < BEACON_MIN_MOVE_M) return;
    lastBeacon = { lat, lon, ts: now };
    postForm('/api/tx/position', {
      lat: lat.toFixed(6), lon: lon.toFixed(6),
      comment: ' via browser', symbol_table: '/', symbol_code: '>',
    }).then((r) => setStatus(r.ok ? `beacon ${lat.toFixed(3)},${lon.toFixed(3)}` : 'beacon failed',
                              r.ok ? 'ok' : 'err'));
  }, (err) => {
    setStatus('GPS error: ' + err.message, 'err');
  }, { enableHighAccuracy:true, maximumAge:5000, timeout:30_000 });
  gpsBtn.classList.add('on');
  setStatus('GPS on (opt-in)', 'ok');
}

// ---------- Map view (lazy-loaded Leaflet) -------------------------------
// We deliberately load Leaflet from a CDN at first use rather than bake it
// into LittleFS — flash on the kv4p-ht board is at 95% already.  If the
// device is on a fully air-gapped network the user can host Leaflet on
// the same LAN and update LEAFLET_BASE below.

const LEAFLET_BASE = 'https://unpkg.com/leaflet@1.9.4/dist';
let leafletReady = null;
let mapInstance = null;
let mapMarkers = {};      // src -> Leaflet marker
let mapTracks  = {};      // src -> Leaflet polyline
let mapVisible = false;

function ensureLeaflet() {
  if (leafletReady) return leafletReady;
  return leafletReady = new Promise((resolve, reject) => {
    const link = document.createElement('link');
    link.rel = 'stylesheet';
    link.href = `${LEAFLET_BASE}/leaflet.css`;
    document.head.appendChild(link);
    const s = document.createElement('script');
    s.src = `${LEAFLET_BASE}/leaflet.js`;
    s.onload  = () => resolve(window.L);
    s.onerror = () => reject(new Error('Leaflet CDN unreachable'));
    document.head.appendChild(s);
  });
}

async function readAllPoints() {
  const db = await trackDb;
  if (!db) return [];
  return new Promise((resolve) => {
    const out = [];
    const tx = db.transaction('points', 'readonly');
    tx.objectStore('points').openCursor().onsuccess = (e) => {
      const c = e.target.result;
      if (!c) return resolve(out);
      out.push(c.value);
      c.continue();
    };
    tx.onerror = () => resolve(out);
  });
}

function colourFor(callsign) {
  // Cheap hash → hue, so each callsign gets a stable colour.
  let h = 0;
  for (let i = 0; i < callsign.length; i++) h = (h * 31 + callsign.charCodeAt(i)) | 0;
  return `hsl(${Math.abs(h) % 360}, 70%, 60%)`;
}

async function populateMap() {
  const L = window.L;
  if (!L || !mapInstance) return;
  const points = await readAllPoints();
  const byCall = {};
  for (const p of points) (byCall[p.src] = byCall[p.src] || []).push(p);
  const all = [];
  for (const call in byCall) {
    const pts = byCall[call].sort((a, b) => a.ts - b.ts);
    const last = pts[pts.length - 1];
    if (last.lat == null || last.lon == null) continue;
    all.push([last.lat, last.lon]);
    if (mapMarkers[call]) {
      mapMarkers[call].setLatLng([last.lat, last.lon]);
    } else {
      mapMarkers[call] = L.circleMarker([last.lat, last.lon], {
        radius: 6, color: colourFor(call), weight: 2,
        fillColor: colourFor(call), fillOpacity: 0.9
      }).addTo(mapInstance).bindPopup(`<b>${call}</b><br>${last.comment || ''}`);
    }
    if (pts.length > 1) {
      const line = pts.map(p => [p.lat, p.lon]);
      if (mapTracks[call]) mapTracks[call].setLatLngs(line);
      else mapTracks[call] = L.polyline(line, {
        color: colourFor(call), weight: 2, opacity: 0.6
      }).addTo(mapInstance);
    }
  }
  const mapStatus = $('#mapStatus');
  mapStatus.textContent = all.length
    ? `${Object.keys(byCall).length} stations · ${points.length} points`
    : 'no positions yet — wait for an APRS position packet';
  if (all.length) mapInstance.fitBounds(all, { padding: [30, 30], maxZoom: 13 });
}

function pushPointToMap(parsed) {
  if (!mapVisible || !window.L || !mapInstance) return;
  if (parsed.lat == null || parsed.lon == null) return;
  const L = window.L;
  const call = parsed.src;
  const ll = [parsed.lat, parsed.lon];
  if (mapMarkers[call]) mapMarkers[call].setLatLng(ll);
  else mapMarkers[call] = L.circleMarker(ll, {
    radius: 6, color: colourFor(call), weight: 2,
    fillColor: colourFor(call), fillOpacity: 0.9
  }).addTo(mapInstance).bindPopup(`<b>${call}</b><br>${parsed.comment || ''}`);
  if (!mapTracks[call]) mapTracks[call] = L.polyline([ll], {
    color: colourFor(call), weight: 2, opacity: 0.6
  }).addTo(mapInstance);
  else mapTracks[call].addLatLng(ll);
}

async function openMapOverlay() {
  $('#mapOverlay').hidden = false;
  mapVisible = true;
  let L;
  try { L = await ensureLeaflet(); }
  catch (e) {
    $('#mapStatus').textContent = 'leaflet load failed (offline?)';
    return;
  }
  if (!mapInstance) {
    mapInstance = L.map('map', { zoomControl: true }).setView([20, 0], 2);
    L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '© OpenStreetMap', maxZoom: 19
    }).addTo(mapInstance);
  }
  setTimeout(() => mapInstance.invalidateSize(), 50);
  await populateMap();
}

function closeMapOverlay() {
  $('#mapOverlay').hidden = true;
  mapVisible = false;
}

// ---------- Wiring --------------------------------------------------------

sendBtn.addEventListener('click', sendCurrentMessage);
msgIn.addEventListener('keydown', (e) => { if (e.key === 'Enter') sendCurrentMessage(); });
gpsBtn.addEventListener('click', toggleGps);
$('#mapBtn').addEventListener('click', openMapOverlay);
$('#mapClose').addEventListener('click', closeMapOverlay);

// ---------- Webhooks panel -----------------------------------------------

async function openHooks() {
  $('#hookOverlay').hidden = false;
  const r = await fetch('/api/webhooks');
  const slots = r.ok ? await r.json() : [];
  const host = $('#hookSlots');
  host.innerHTML = '';
  slots.forEach(renderHookSlot);
}
function closeHooks() { $('#hookOverlay').hidden = true; }

function renderHookSlot(s) {
  const host = $('#hookSlots');
  const wrap = document.createElement('div');
  wrap.className = 'hook-slot';
  const id = (k) => `h${s.slot}_${k}`;
  wrap.innerHTML = `
    <div class="row">
      <label style="flex:0 0 auto"><input type="checkbox" id="${id('en')}" ${s.enabled?'checked':''}> enabled</label>
      <label class="grow">name<input type="text" id="${id('name')}" value="${escapeAttr(s.name)}" placeholder="Slot ${s.slot+1}"></label>
    </div>
    <label>URL <input type="text" id="${id('url')}" value="${escapeAttr(s.url)}" placeholder="https://api.telegram.org/bot…/sendMessage"></label>
    <label>JSON body template (empty = default) <textarea id="${id('body')}" placeholder='{"chat_id":"123","text":"{src}: {message}"}'>${escapeText(s.body_template)}</textarea></label>
    <div class="row">
      <label class="grow">filter callsign (prefix, blank = all) <input type="text" id="${id('flt')}" value="${escapeAttr(s.filter_callsign)}" placeholder="e.g. M0XYZ"></label>
      <label style="flex:0 0 auto">events<select id="${id('evt')}">
        <option value="1" ${s.event_mask===1?'selected':''}>any RX</option>
        <option value="2" ${s.event_mask===2?'selected':''}>messages to me</option>
        <option value="4" ${s.event_mask===4?'selected':''}>positions only</option>
        <option value="8" ${s.event_mask===8?'selected':''}>status only</option>
        <option value="3" ${s.event_mask===3?'selected':''}>any + msgs to me</option>
      </select></label>
    </div>
    <div class="row">
      <button type="button" data-act="save" data-slot="${s.slot}">Save</button>
      <button type="button" data-act="test" data-slot="${s.slot}">Send test</button>
      <span class="res" id="${id('res')}"></span>
    </div>`;
  host.appendChild(wrap);
}

function escapeAttr(s) { return String(s||'').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c])); }
function escapeText(s) { return escapeAttr(s); }

document.addEventListener('click', async (e) => {
  const btn = e.target.closest('button[data-act]');
  if (!btn) return;
  const slot = +btn.dataset.slot;
  const id = (k) => `h${slot}_${k}`;
  const res = document.getElementById(id('res'));
  if (btn.dataset.act === 'save') {
    res.textContent = 'saving…'; res.className='res';
    const r = await postForm('/api/webhooks', {
      slot,
      enabled:    document.getElementById(id('en')).checked ? '1' : '0',
      event_mask: document.getElementById(id('evt')).value,
      name:       document.getElementById(id('name')).value,
      url:        document.getElementById(id('url')).value,
      body_template: document.getElementById(id('body')).value,
      filter_callsign: document.getElementById(id('flt')).value,
    });
    res.textContent = r.ok ? 'saved' : 'save failed';
    res.className = 'res ' + (r.ok ? 'ok' : 'err');
  } else if (btn.dataset.act === 'test') {
    res.textContent = 'queueing…'; res.className='res';
    const r = await postForm('/api/webhooks/test', { slot });
    const j = r.body || {};
    if (j.queued) {
      res.textContent = 'queued — check destination';
      res.className = 'res ok';
    } else {
      res.textContent = j.err || 'failed';
      res.className = 'res err';
    }
  }
});

$('#hookBtn').addEventListener('click', openHooks);
$('#hookClose').addEventListener('click', closeHooks);

// ---------- FM voice (Phase 5 — scaffolding only) -------------------------
// The firmware already exposes /ws_audio on port 81 (see webservice.cpp's
// onWsEvent).  The intended browser-side flow is:
//   • RX monitor: server sends 8 kHz mono i16 LE PCM frames; browser plays
//     them via an AudioWorklet so the audio stream never blocks the main
//     thread.  Simple decimation/filtering happens in the worklet.
//   • TX (PTT):  browser captures mic via getUserMedia, downsamples to 8
//     kHz mono, sends binary frames; firmware writes to DAC and toggles PTT.
// All DSP (AGC, VAD, level meter, optional Opus encoding) runs in the
// browser so the ESP32 only has to push bytes around.
//
// We deliberately do not auto-connect the audio WS yet — wiring it up
// should land in a follow-up change with a proper PTT button + worklet.
const VoiceFM = {
  ws: null,
  startMonitor() { /* TODO: connect ws://${host}:81/ws_audio, play PCM */ },
  stopMonitor()  { /* TODO */ },
  pttDown()      { /* TODO: getUserMedia → AudioWorklet → ws.send(pcm) */ },
  pttUp()        { /* TODO */ },
};
window.VoiceFM = VoiceFM; // exposed for future UI binding

// ---------- Station sheet (callsign + frequency) -------------------------

function openStation() {
  $('#stationOverlay').hidden = false;
  $('#stCall').value   = me.callsign || '';
  $('#stSsid').value   = me.ssid     || 0;
  $('#stFreq').value   = (radio.freq_rx ?? '').toString();
  $('#stFreqRx').value = (radio.freq_rx ?? '').toString();
  $('#stFreqTx').value = (radio.freq_tx ?? '').toString();
  $('#stSql').value    = radio.sql_level ?? 0;
  $('#stToneRx').value = radio.tone_rx ?? 0;
  $('#stToneTx').value = radio.tone_tx ?? 0;
  $('#stPwr').value    = radio.rf_power ? '1' : '0';
  $('#stRfEn').checked = !!radio.rf_en;
  $('#stIdRes').textContent = '';
  $('#stRadioRes').textContent = '';
}
function closeStation() { $('#stationOverlay').hidden = true; }

async function saveIdentity() {
  const res = $('#stIdRes');
  res.textContent = 'saving…'; res.className = 'res';
  const r = await postForm('/api/identity', {
    callsign: $('#stCall').value,
    ssid:     $('#stSsid').value || '0',
  });
  if (r.ok) {
    res.textContent = 'saved'; res.className = 'res ok';
    await loadMe();
  } else {
    res.textContent = (r.body && r.body.err) || 'failed';
    res.className = 'res err';
  }
}

async function saveRadio() {
  const res = $('#stRadioRes');
  res.textContent = 'applying…'; res.className = 'res';
  // Prefer the simplex single-freq field when filled; otherwise send split.
  const fields = {
    sql_level: $('#stSql').value,
    rf_power:  $('#stPwr').value,
    tone_rx:   $('#stToneRx').value,
    tone_tx:   $('#stToneTx').value,
    rf_en:     $('#stRfEn').checked ? '1' : '0',
  };
  if ($('#stFreq').value) {
    fields.freq = $('#stFreq').value;
  } else {
    fields.freq_rx = $('#stFreqRx').value;
    fields.freq_tx = $('#stFreqTx').value;
  }
  const r = await postForm('/api/radio', fields);
  if (r.ok) {
    res.textContent = 'applied (radio re-init queued)';
    res.className = 'res ok';
    await loadRadio();
  } else {
    res.textContent = (r.body && r.body.err) || 'failed';
    res.className = 'res err';
  }
}

$('#stationBtn').addEventListener('click', openStation);
$('#stationClose').addEventListener('click', closeStation);
$('#stSaveId').addEventListener('click', saveIdentity);
$('#stSaveRadio').addEventListener('click', saveRadio);
// Frequency presets
document.addEventListener('click', (e) => {
  const b = e.target.closest('#stationOverlay .preset');
  if (!b) return;
  $('#stFreq').value   = b.dataset.freq;
  $('#stFreqRx').value = '';
  $('#stFreqTx').value = '';
});

(async () => {
  setStatus('loading…', 'dim');
  await Promise.all([loadMe(), loadRadio()]);
  await hydrate();
  connectStream();
  // /api/me changes slowly, /api/radio carries the live audio-level meter so
  // refresh it more often.
  setInterval(loadMe,    30_000);
  setInterval(loadRadio,  3_000);
})();

})();
