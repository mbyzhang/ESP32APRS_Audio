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
const pathBtn= $('#pathBtn');
const pathMenu = $('#pathMenu');
const gpsBtn = $('#gpsBtn');

let me = { callsign: '', ssid: 0 };
const seen = new Set();          // dedupe by raw+ts
const trackDb = openTrackDb();   // opened lazily, IndexedDB; future Phase 3
const PATH_OPTIONS = [
  { value: '', label: 'Direct', help: 'No digipeater path. Best for nearby stations or bench testing.' },
  { value: 'WIDE1-1', label: 'WIDE1-1', help: 'One local hop. Good for handhelds and mobiles near a fill-in digipeater.' },
  { value: 'WIDE2-1', label: 'WIDE2-1', help: 'One wide-area hop. Common for a home station with a good antenna.' },
  { value: 'WIDE1-1,WIDE2-1', label: 'WIDE1-1,WIDE2-1', help: 'Typical two-hop mobile path: local fill-in first, then one wide hop.' },
  { value: 'WIDE1-0,WIDE2-1', label: 'WIDE1-0,WIDE2-1', help: 'Shows the WIDE1 hop as already used, then allows one WIDE2 hop.' },
  { value: 'RFONLY', label: 'RFONLY', help: 'Ask internet gateways not to forward this to APRS-IS.' },
  { value: 'RFONLY-0', label: 'RFONLY-0', help: 'RFONLY with explicit SSID-0 for equipment that expects that form.' },
];
let selectedPath = localStorage.getItem('aprs-path') || 'WIDE1-1';

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
  // Compressed?  First char in [!-{] but not a digit.
  const first = info.charCodeAt(0);
  if (info.length >= 10 && first >= 0x21 && first <= 0x7b && !/[0-9]/.test(info[0])) {
    // Compressed: SYM_TABLE LAT(4) LON(4) SYM_CODE [optional CS(2) T(1)] COMMENT
    const symT = info[0];
    const lat = 90  - decodeBase91(info.slice(1, 5))  / 380926;
    const lon = -180 + decodeBase91(info.slice(5, 9))  / 190463;
    const symC = info[9];
    const comment = info.length > 13 ? info.slice(13) : '';
    return { lat, lon, symbol_table: symT, symbol_code: symC, comment };
  }
  if (info.length < 19) return {};
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
    // bumped to v2 to add the "packets" store — keeps the chat timeline
    // across page refreshes (incl. our own self-TX which the device never
    // re-broadcasts to /api/packets/recent).
    const r = indexedDB.open('aprs-tracks', 2);
    r.onupgradeneeded = (ev) => {
      const db = r.result;
      if (!db.objectStoreNames.contains('points'))
        db.createObjectStore('points', { keyPath: 'id', autoIncrement: true })
          .createIndex('byCall', 'src');
      if (!db.objectStoreNames.contains('packets'))
        db.createObjectStore('packets', { keyPath: 'id', autoIncrement: true })
          .createIndex('byTs', 'ts');
    };
    r.onsuccess = () => resolve(r.result);
    r.onerror   = () => resolve(null);
  });
}

const PACKETS_KEEP = 500;   // upper bound on persisted packet history

async function persistPacket(pkt) {
  const db = await trackDb;
  if (!db) return;
  try {
    const tx = db.transaction('packets', 'readwrite');
    const store = tx.objectStore('packets');
    store.add({ ts: pkt.ts, ch: pkt.ch, audio: pkt.audio, dir: pkt.dir || 'rx', raw: pkt.raw });
    // Trim oldest entries — count first, then delete from the front of the
    // byTs index until we're below the cap.
    const countReq = store.count();
    countReq.onsuccess = () => {
      const overflow = countReq.result - PACKETS_KEEP;
      if (overflow <= 0) return;
      const idx = store.index('byTs');
      const cur = idx.openCursor();
      let removed = 0;
      cur.onsuccess = (ev) => {
        const c = ev.target.result;
        if (!c || removed >= overflow) return;
        c.delete();
        removed++;
        c.continue();
      };
    };
  } catch (_) {}
}

async function loadPersistedPackets(limit = PACKETS_KEEP) {
  const db = await trackDb;
  if (!db) return [];
  return new Promise((resolve) => {
    const out = [];
    try {
      const tx = db.transaction('packets', 'readonly');
      const idx = tx.objectStore('packets').index('byTs');
      idx.openCursor().onsuccess = (ev) => {
        const c = ev.target.result;
        if (!c) return resolve(out);
        out.push(c.value);
        if (out.length >= limit) return resolve(out);
        c.continue();
      };
      tx.onerror = () => resolve(out);
    } catch (_) { resolve(out); }
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

function osmTileFor(lat, lon, z = 13) {
  const n = 2 ** z;
  const x = (lon + 180) / 360 * n;
  const latRad = lat * Math.PI / 180;
  const y = (1 - Math.log(Math.tan(latRad) + 1 / Math.cos(latRad)) / Math.PI) / 2 * n;
  return {
    z,
    x: Math.floor(x),
    y: Math.floor(y),
    px: Math.round((x - Math.floor(x)) * 100),
    py: Math.round((y - Math.floor(y)) * 100),
  };
}

function renderPositionBody(parsed) {
  if (parsed.lat == null || parsed.lon == null) {
    return escapeHtml(parsed.comment || parsed.info || parsed.raw || '');
  }
  const lat = Number(parsed.lat);
  const lon = Number(parsed.lon);
  const coords = `${lat.toFixed(5)}, ${lon.toFixed(5)}`;
  const comment = parsed.comment ? `<div>${escapeHtml(parsed.comment)}</div>` : '';
  const tile = osmTileFor(lat, lon);
  const osmUrl = `https://www.openstreetmap.org/?mlat=${lat}&mlon=${lon}#map=14/${lat}/${lon}`;
  const tileUrl = `https://tile.openstreetmap.org/${tile.z}/${tile.x}/${tile.y}.png`;
  return `${comment}
    <a class="mini-map" target="_blank" rel="noopener" href="${osmUrl}" style="background-image:url('${tileUrl}')">
      <span class="pin" style="left:${tile.px}%;top:${tile.py}%"></span>
    </a>
    <a class="coords" target="_blank" rel="noopener" href="${osmUrl}">${coords}</a>`;
}

// ---------- Filter (browser-side, applied to renderPacket) ---------------
// The chat UI displays *every* packet the firmware reports.  An optional
// filter narrows what the user sees on the screen — server-side state is
// untouched, so toggling the filter never loses traffic from the SSE.
const filterState = { text: '', mode: 'any' };
const allPackets  = [];      // ring of every packet we've seen, for re-filter
const ALL_MAX     = 600;

function packetMatchesFilter(parsed) {
  // Never hide our own outbound messages; delivery status has to remain
  // visible even while the user is filtering the receive feed.
  if (parsed.dir === 'tx') return true;
  const q = filterState.text;
  if (!q) return true;
  const me = (fullCall() || '').toUpperCase();
  const src = (parsed.src || '').toUpperCase();
  const dst = (parsed.addressee || parsed.dst || '').toUpperCase();
  const txt = (parsed.message || parsed.comment || parsed.info || '').toUpperCase();
  switch (filterState.mode) {
    case 'src':  return src.startsWith(q) || src.includes(q);
    case 'dst':  return dst.startsWith(q) || dst.includes(q);
    case 'text': return txt.includes(q);
    case 'me':   return src === me || dst === me ||
                        src.startsWith(me) || dst.startsWith(me);
    case 'any':
    default:     return src.includes(q) || dst.includes(q) || txt.includes(q);
  }
}

function applyFilterToFeed() {
  // Cheap full re-render from the in-memory ring; preserves scroll position.
  const wasAtBottom = isAtBottom();
  feed.innerHTML = '';
  for (const pkt of allPackets) renderPacketInternal(pkt, /*recordOnly*/false);
  if (wasAtBottom) feed.scrollTop = feed.scrollHeight;
}

function renderPacket(pkt, opts = {}) {
  const key = pkt.ts + '|' + pkt.raw;
  if (seen.has(key)) return;
  seen.add(key);
  if (seen.size > 2000) {
    const drop = seen.values().next().value;
    seen.delete(drop);
  }
  allPackets.push(pkt);
  if (allPackets.length > ALL_MAX) allPackets.shift();
  renderPacketInternal(pkt, true);
  // Persist for the next page load.  We skip when re-rendering historical
  // entries (opts.skipPersist) so we don't double-write on hydration.
  if (!opts.skipPersist) persistPacket(pkt);
}

function renderPacketInternal(pkt, recordSideEffects) {
  const parsed = parseTnc2(pkt.raw) || { src:'?', type:'unknown' };
  parsed.ts = pkt.ts;
  parsed.audio = pkt.audio;
  parsed.ch = pkt.ch;
  parsed.dir = pkt.dir || 'rx';

  // Side effects (track DB, map plot) happen on first render only — we don't
  // want to re-record points every time the filter is toggled.
  if (recordSideEffects) recordTrackPoint(parsed);

  if (!packetMatchesFilter(parsed)) return;

  const wasAtBottom = isAtBottom();

  const el = document.createElement('div');
  el.className = 'msg';
  // Treat anything tagged dir=tx OR matching our own callsign as "me".
  const isMine = parsed.dir === 'tx' || parsed.src === fullCall();
  if (isMine) el.classList.add('me');
  if (parsed.type === 'message' && parsed.addressee === fullCall()) el.classList.add('dm');

  const isMsg = parsed.type === 'message';
  if (parsed.dir !== 'tx' && isMsg) {
    const repeatKey = [
      parsed.src || '',
      parsed.addressee || '',
      parsed.msgid || parsed.message || '',
    ].join('|');
    const existing = feed.querySelector(`.msg[data-rxmsgkey="${CSS.escape(repeatKey)}"]`);
    if (existing) {
      const count = (parseInt(existing.dataset.repeat || '1', 10) || 1) + 1;
      existing.dataset.repeat = String(count);
      const statusEl = existing.querySelector('.status');
      if (statusEl) statusEl.textContent = `${headLabelForPacket(pkt)} · x${count}`;
      const tsEl = existing.querySelector('.ts');
      if (tsEl) tsEl.firstChild.textContent = fmtTime(pkt.ts) + ' · ';
      if (wasAtBottom) feed.scrollTop = feed.scrollHeight;
      return;
    }
    el.dataset.rxmsgkey = repeatKey;
    el.dataset.repeat = '1';
  }
  if (parsed.dir === 'tx' && isMsg && parsed.msgid) {
    const existing = feed.querySelector(`.msg.me[data-msgid="${CSS.escape(String(parsed.msgid))}"]`);
    if (existing) return;
  }
  const headRight = headLabelForPacket(pkt);

  let body = '';
  if (isMsg) {
    body = `<span class="dim">→ ${escapeHtml(parsed.addressee)}:</span> ${escapeHtml(parsed.message || '')}`;
  } else if (parsed.type === 'position') {
    body = renderPositionBody(parsed);
  } else if (parsed.type === 'status') {
    body = escapeHtml(parsed.status);
  } else {
    body = escapeHtml(parsed.info || pkt.raw);
  }

  // Mark TX message bubbles with their APRS msgID so the pending-status
  // poller can later flip the badge from "sent" → "✓✓ ack'd" / "retry 2/3"
  // / "failed".  The msgID comes out of parseTnc2's parsing of the {NNN
  // trailer on the message line.
  if (parsed.dir === 'tx' && isMsg && parsed.msgid) {
    el.dataset.msgid = String(parsed.msgid);
    el.dataset.status = 'sent';
  }

  el.innerHTML =
    `<div class="head">
       <span class="src">${escapeHtml(parsed.src)}</span>
       <span class="ts">${fmtTime(pkt.ts)} · <span class="status">${escapeHtml(headRight)}</span></span>
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
}

function headLabelForPacket(pkt) {
  return pkt.dir === 'tx'
    ? '✓ sent'
    : (pkt.ch === 1 ? 'IS' : (pkt.audio ? `${pkt.audio} dBV` : 'RF'));
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

async function syncBrowserTime() {
  try {
    const now = Date.now();
    const tzHours = -new Date(now).getTimezoneOffset() / 60;
    const res = await postForm('/api/time', {
      epoch: Math.floor(now / 1000),
      tz: tzHours.toFixed(2),
    });
    if (res.ok) setStatus('time synced', 'ok');
  } catch (_) {}
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
    // Show freq + audio level + modem state so the user can debug "no RX"
    // at a glance.
    //   mvrms : running RMS of demod input (mV).  Stays low (~8) in silence,
    //           jumps to several hundred on real signal.
    //   dcd   : demodulator carrier-detect counter (0..100).  Must rise
    //           above 3 for the AFSK decoder to even attempt to decode.
    //   modem : 1 = Bell 202 (APRS).  Anything else means RX is silently
    //           broken until you change it from the Station sheet.
    const mv = radio.mvrms ?? 0;
    const dcd = radio.dcd ?? 0;
    const sq = radio.sql_pin === 0 ? ' · sq' : '';
    const modemBad = (radio.modem ?? 1) !== 1;
    let label;
    if (!radio.rf_en) label = `${f} MHz · RF off`;
    else if (modemBad) label = `${f} MHz · WRONG MODEM (${radio.modem})`;
    else                label = `${f} MHz · ${mv} mV · dcd ${dcd}${sq}`;
    $('#meFreq').textContent = label;
    $('#meFreq').classList.toggle('warn', !radio.rf_en || modemBad);
  } catch (_) {}
}
function fullCall() {
  return me.ssid ? `${me.callsign}-${me.ssid}` : me.callsign;
}

async function hydrate() {
  // 1. Replay persisted packets first (includes our self-TX, which the
  //    device never re-broadcasts).
  try {
    const persisted = await loadPersistedPackets();
    // already in chronological order from byTs index
    for (const pkt of persisted) renderPacket(pkt, { skipPersist: true });
  } catch (_) {}

  // 2. Then ask the device for its in-RAM RX ring; dedupe is handled by the
  //    `seen` Set inside renderPacket().
  try {
    const r = await fetch('/api/packets/recent');
    if (r.ok) {
      const arr = await r.json();
      arr.reverse().forEach((pkt) => renderPacket(pkt));
    }
  } catch (_) {}

  feed.scrollTop = feed.scrollHeight;
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

function currentPathOption() {
  return PATH_OPTIONS.find((p) => p.value === selectedPath) || PATH_OPTIONS[1];
}

function updatePathButton() {
  const opt = currentPathOption();
  selectedPath = opt.value;
  pathBtn.textContent = opt.label;
  pathBtn.title = `APRS path: ${opt.label}`;
  pathBtn.setAttribute('aria-expanded', pathMenu.hidden ? 'false' : 'true');
}

function renderPathMenu() {
  pathMenu.innerHTML = PATH_OPTIONS.map((p) => `
    <button class="path-option ${p.value === selectedPath ? 'on' : ''}" type="button" data-path="${escapeHtml(p.value)}">
      <b>${escapeHtml(p.label)}</b>
      <span>${escapeHtml(p.help)}</span>
    </button>
  `).join('');
}

function setSelectedPath(path) {
  selectedPath = PATH_OPTIONS.some((p) => p.value === path) ? path : 'WIDE1-1';
  localStorage.setItem('aprs-path', selectedPath);
  renderPathMenu();
  updatePathButton();
}

function togglePathMenu(forceOpen) {
  const open = forceOpen == null ? pathMenu.hidden : forceOpen;
  pathMenu.hidden = !open;
  pathBtn.setAttribute('aria-expanded', open ? 'true' : 'false');
  if (open) renderPathMenu();
}

async function sendCurrentMessage() {
  const to   = (toCall.value || '').trim().toUpperCase();
  const text = (msgIn.value  || '').trim();
  if (!to || !text) return;
  sendBtn.disabled = true;
  setStatus('sending…', 'warn');
  const res = await postForm('/api/tx/message', { to, text, path: selectedPath });
  sendBtn.disabled = false;
  if (res.ok) {
    setStatus('sent', 'ok');
    if (res.body?.msgID) {
      renderPendingMessage({
        msgID: res.body.msgID,
        to,
        text,
        status: 'pending',
        ack: null,
        ts: Math.floor(Date.now() / 1000),
        retries_left: null,
        retries_total: null,
        path: selectedPath,
      });
    }
    msgIn.value = '';
    // Kick the pending-status poller so the just-sent bubble gets its
    // retry/ack badge populated quickly instead of waiting for the next
    // periodic tick.
    setTimeout(pollPendingMessages, 400);
  } else {
    setStatus('send failed (' + res.status + ')', 'err');
  }
}

// ---------- Outgoing-message retry / ack status -------------------------
// The firmware keeps a queue of unACK'd outbound messages.  Every few
// seconds we ask /api/messages/pending what state they're in and patch
// the corresponding TX bubbles in the feed.

async function pollPendingMessages() {
  let list;
  try {
    const r = await fetch('/api/messages/pending');
    if (!r.ok) return;
    list = await r.json();
  } catch (_) { return; }

  // Index by msgID so we can find the matching bubble quickly.
  const byId = new Map();
  for (const m of list) byId.set(String(m.msgID), m);

  for (const entry of list) renderPendingMessage(entry);

  // Walk every TX bubble that's still pending.
  for (const el of feed.querySelectorAll('.msg.me[data-msgid]')) {
    const id = el.dataset.msgid;
    const entry = byId.get(id);
    const statusEl = el.querySelector('.status');
    if (!statusEl) continue;
    if (!entry) {
      // Server forgot about it — likely fell out of the small ring.  Leave
      // whatever we last showed.
      continue;
    }
    let label, cls;
    if (entry.status === 'acked') {
      label = '✓✓ ack';
      cls   = 'ok';
    } else if (entry.status === 'failed') {
      label = '✗ failed';
      cls   = 'err';
    } else {
      const used = entry.retries_total - entry.retries_left + 1;
      label = `retry ${used}/${entry.retries_total}`;
      cls   = 'warn';
    }
    statusEl.textContent = label;
    statusEl.className = 'status ' + cls;
    el.dataset.status = entry.status;
  }
}

function txRawFromPending(entry) {
  const src = fullCall() || me.callsign || 'NOCALL';
  const to = String(entry.to || '').toUpperCase().padEnd(9, ' ').slice(0, 9);
  const text = String(entry.text || '');
  const path = entry.path ? `,${entry.path}` : '';
  return `${src}>APE32L${path}::${to}:${text}{${entry.msgID}`;
}

function renderPendingMessage(entry) {
  if (!entry || entry.msgID == null) return;
  const id = String(entry.msgID);
  if (!feed.querySelector(`.msg.me[data-msgid="${CSS.escape(id)}"]`)) {
    renderPacket({
      ts: entry.ts || Math.floor(Date.now() / 1000),
      ch: 0,
      audio: 0,
      dir: 'tx',
      raw: txRawFromPending(entry),
    });
  }
  updatePendingMessageStatus(entry);
}

function updatePendingMessageStatus(entry) {
  if (!entry || entry.msgID == null) return;
  const el = feed.querySelector(`.msg.me[data-msgid="${CSS.escape(String(entry.msgID))}"]`);
  if (!el) return;
  const statusEl = el.querySelector('.status');
  if (!statusEl) return;
  let label, cls;
  if (entry.status === 'acked') {
    label = '✓✓ ack';
    cls   = 'ok';
  } else if (entry.status === 'failed') {
    label = '✗ failed';
    cls   = 'err';
  } else {
    const total = Number(entry.retries_total ?? 0);
    const left = Number(entry.retries_left ?? total);
    const used = total > 0 ? Math.max(1, total - left + 1) : 1;
    label = total > 0 ? `retry ${used}/${total}` : 'pending';
    cls   = 'warn';
  }
  statusEl.textContent = label;
  statusEl.className = 'status ' + cls;
  el.dataset.status = entry.status || 'pending';
}

// Poll while page is open.  Cheap: a single small GET on a 3-second cadence.
setInterval(pollPendingMessages, 3000);

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
  exitPickMode();
}

// ---------- Pick-on-map → send position ---------------------------------
// Tap the "send from map" button on the map overlay → tap anywhere on the
// map → confirm bar at the bottom shows the lat/lon and a Send button which
// POSTs to /api/tx/position.  Useful for relaying someone else's location
// or for setting a static beacon site without GPS.
let mapPickMarker = null;
let mapPickLatLng = null;
let mapPickClickHandler = null;

function enterPickMode() {
  if (!mapInstance) return;
  $('#mapPickBtn').classList.add('on');
  $('#mapPickBar').hidden = false;
  $('#mapPickCoords').textContent = 'tap the map to place a marker';
  $('#mapPickSend').disabled = true;
  mapPickClickHandler = (e) => {
    mapPickLatLng = e.latlng;
    const L = window.L;
    if (mapPickMarker) {
      mapPickMarker.setLatLng(e.latlng);
    } else {
      mapPickMarker = L.marker(e.latlng, { draggable: true }).addTo(mapInstance);
      mapPickMarker.on('dragend', (ev) => {
        mapPickLatLng = ev.target.getLatLng();
        updatePickCoords();
      });
    }
    updatePickCoords();
  };
  mapInstance.on('click', mapPickClickHandler);
}

function updatePickCoords() {
  if (!mapPickLatLng) return;
  $('#mapPickCoords').textContent =
    `${mapPickLatLng.lat.toFixed(5)}, ${mapPickLatLng.lng.toFixed(5)}`;
  $('#mapPickSend').disabled = false;
}

function exitPickMode() {
  $('#mapPickBtn').classList.remove('on');
  $('#mapPickBar').hidden = true;
  if (mapInstance && mapPickClickHandler) {
    mapInstance.off('click', mapPickClickHandler);
    mapPickClickHandler = null;
  }
  if (mapPickMarker) {
    mapPickMarker.remove();
    mapPickMarker = null;
  }
  mapPickLatLng = null;
}

async function sendPickedLocation() {
  if (!mapPickLatLng) return;
  const lat = mapPickLatLng.lat;
  const lon = mapPickLatLng.lng;
  $('#mapPickSend').disabled = true;
  $('#mapPickCoords').textContent = 'sending…';
  const r = await postForm('/api/tx/position', {
    lat: lat.toFixed(6),
    lon: lon.toFixed(6),
    comment: ' picked from map',
    path: selectedPath,
    symbol_table: '/', symbol_code: '>',
  });
  if (r.ok) {
    $('#mapPickCoords').textContent =
      `✓ sent ${lat.toFixed(4)}, ${lon.toFixed(4)}`;
    setTimeout(exitPickMode, 1500);
  } else {
    $('#mapPickCoords').textContent = 'send failed';
    $('#mapPickSend').disabled = false;
  }
}

// ---------- Wiring --------------------------------------------------------

setSelectedPath(selectedPath);
pathBtn.addEventListener('click', () => togglePathMenu());
pathMenu.addEventListener('click', (e) => {
  const opt = e.target.closest('.path-option');
  if (!opt) return;
  setSelectedPath(opt.dataset.path || '');
  togglePathMenu(false);
});
document.addEventListener('click', (e) => {
  if (pathMenu.hidden) return;
  if (pathMenu.contains(e.target) || pathBtn.contains(e.target)) return;
  togglePathMenu(false);
});
document.addEventListener('keydown', (e) => {
  if (e.key === 'Escape') togglePathMenu(false);
});
sendBtn.addEventListener('click', sendCurrentMessage);
msgIn.addEventListener('keydown', (e) => { if (e.key === 'Enter') sendCurrentMessage(); });
gpsBtn.addEventListener('click', toggleGps);
$('#mapBtn').addEventListener('click', openMapOverlay);
$('#mapClose').addEventListener('click', closeMapOverlay);
$('#mapPickBtn').addEventListener('click', () => {
  if ($('#mapPickBtn').classList.contains('on')) exitPickMode();
  else enterPickMode();
});
$('#mapPickSend').addEventListener('click', sendPickedLocation);
$('#mapPickCancel').addEventListener('click', exitPickMode);

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

// ---------- FM voice — RX monitor + TX PTT --------------------------------
// Talks to /ws_audio on port 81 (see src/webservice.cpp:onWsEvent).
// Wire format:
//   server → browser (binary): μ-law @ 8 kHz mono, 320-byte chunks
//   server → browser (text):   JSON {type:"cfg"|"tx"|...}
//   browser → server (text):   "tx_start" | "tx_stop" | "ping" | "set_freq:tx,rx"
//   browser → server (binary): μ-law @ 8 kHz mono mic capture
//
// All resampling and μ-law conversion runs in the browser via a plain
// ScriptProcessorNode.  AudioWorklet would be cleaner but ScriptProcessor
// works on every browser/OS combo without an extra worklet file in
// LittleFS — important when flash is the scarce resource.

const VoiceFM = (() => {
  const state = {
    ws: null,
    monitoring: false,
    txActive: false,
    muted: false,
    sampleRate: 8000,        // server-pushed; usually 8000
    audioCtx: null,
    gainNode: null,
    procNode: null,
    queue: [],               // Float32Array chunks awaiting playback
    queueOffset: 0,
    queuedSamples: 0,
    lastSample: 0,
    resampleAcc: 0,
    rxLevel: 0,
    txLevel: 0,
    micStream: null,
    micCtx: null,
    micProc: null,
  };

  // ---------- μ-law (G.711) <-> linear int16 ------------------------------
  function mulawToLinear(u) {
    u = (~u) & 0xFF;
    const sign = u & 0x80;
    const exponent = (u >> 4) & 0x07;
    const mantissa = u & 0x0F;
    let s = ((mantissa << 3) + 0x84) << exponent;
    s -= 0x84;
    return sign ? -s : s;
  }
  function linearToMulaw(v) {
    let pcm = Math.max(-1, Math.min(1, v));
    pcm = (pcm * 32767) | 0;
    const sign = (pcm < 0) ? 0x80 : 0;
    if (pcm < 0) pcm = -pcm;
    if (pcm > 32635) pcm = 32635;
    pcm += 0x84;
    let exp = 7;
    for (let m = 0x4000; (pcm & m) === 0 && exp > 0; m >>= 1) exp--;
    const mant = (pcm >> (exp + 3)) & 0x0F;
    return (~(sign | (exp << 4) | mant)) & 0xFF;
  }

  function clearQueue() {
    state.queue = [];
    state.queueOffset = 0;
    state.queuedSamples = 0;
    state.lastSample = 0;
    state.resampleAcc = 0;
  }

  function popSample() {
    if (state.queue.length === 0) return 0;
    const c = state.queue[0];
    const v = c[state.queueOffset++];
    state.queuedSamples--;
    if (state.queueOffset >= c.length) {
      state.queue.shift();
      state.queueOffset = 0;
    }
    return v;
  }

  function ensureAudioPath() {
    if (state.audioCtx) return;
    const Ctx = window.AudioContext || window.webkitAudioContext;
    state.audioCtx = new Ctx();
    state.gainNode = state.audioCtx.createGain();
    state.procNode = state.audioCtx.createScriptProcessor(1024, 1, 1);
    state.procNode.onaudioprocess = (e) => {
      const out = e.outputBuffer.getChannelData(0);
      const outRate = state.audioCtx.sampleRate;
      let peak = 0;
      for (let i = 0; i < out.length; i++) {
        state.resampleAcc += state.sampleRate;
        while (state.resampleAcc >= outRate) {
          state.lastSample = popSample();
          state.resampleAcc -= outRate;
        }
        const s = state.lastSample;
        out[i] = s;
        const a = Math.abs(s);
        if (a > peak) peak = a;
      }
      // Decay-tracked peak for VU-style meter
      state.rxLevel = Math.max(state.rxLevel * 0.85, peak);
    };
    state.procNode.connect(state.gainNode);
    state.gainNode.connect(state.audioCtx.destination);
    setVolumePercent(parseInt($('#vpVol').value, 10) || 80);
  }

  function setVolumePercent(p) {
    const g = Math.max(0, Math.min(1, p / 100));
    if (state.gainNode) state.gainNode.gain.value = state.muted ? 0 : g;
  }

  function setStatus(txt, cls) {
    const el = $('#vpStatus');
    if (!el) return;
    el.textContent = txt;
    el.className = cls || 'dim';
  }

  function wsUrl() {
    const proto = (window.location.protocol === 'https:') ? 'wss://' : 'ws://';
    // The audio WS lives on port 81 (async_websocket).  Use explicit port
    // so the chat UI (port 80) can reach it from the same origin.
    return `${proto}${location.hostname}:81/ws_audio`;
  }

  async function startMonitor() {
    if (state.monitoring) return;
    ensureAudioPath();
    try { await state.audioCtx.resume(); } catch (_) {}
    clearQueue();
    state.ws = new WebSocket(wsUrl());
    state.ws.binaryType = 'arraybuffer';
    state.ws.onopen = () => {
      state.monitoring = true;
      $('#vpToggle').classList.add('on');
      $('#vpToggle').textContent = 'Stop';
      setStatus('● live', 'ok');
    };
    state.ws.onclose = () => {
      const wasOn = state.monitoring;
      cleanupWs();
      if (wasOn) setStatus('disconnected', 'warn');
    };
    state.ws.onerror = () => setStatus('socket error', 'err');
    state.ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') {
        try {
          const msg = JSON.parse(ev.data);
          if (msg.type === 'cfg' && msg.rate) {
            state.sampleRate = parseInt(msg.rate, 10);
          } else if (msg.type === 'tx' && msg.state === 'off') {
            // Server forced TX off (e.g. another client took it)
            stopTx(/*localOnly*/true);
          }
        } catch (_) {}
        return;
      }
      const ulaw = new Uint8Array(ev.data);
      const pcm = new Float32Array(ulaw.length);
      for (let i = 0; i < ulaw.length; i++) pcm[i] = mulawToLinear(ulaw[i]) / 32768;
      state.queue.push(pcm);
      state.queuedSamples += pcm.length;
      // Cap latency at ~2 s so we don't drift forever on a slow link
      const max = state.sampleRate * 2;
      while (state.queuedSamples > max && state.queue.length) {
        state.queuedSamples -= state.queue[0].length;
        state.queue.shift();
        state.queueOffset = 0;
      }
    };
  }

  function cleanupWs() {
    if (state.ws) {
      try { state.ws.onopen = state.ws.onclose = state.ws.onmessage = state.ws.onerror = null; } catch (_) {}
      try { state.ws.close(); } catch (_) {}
      state.ws = null;
    }
    state.monitoring = false;
    state.txActive = false;
    $('#vpToggle').classList.remove('on');
    $('#vpToggle').textContent = 'Listen';
    $('#vpPtt').classList.remove('live');
  }

  function stopMonitor() {
    if (!state.monitoring && !state.ws) return;
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
      try { state.ws.send('tx_stop'); } catch (_) {}
    }
    cleanupWs();
    clearQueue();
    if (state.audioCtx && state.audioCtx.state !== 'closed') {
      state.audioCtx.suspend().catch(() => {});
    }
    setStatus('idle');
  }

  async function ensureMicPath() {
    if (!navigator.mediaDevices?.getUserMedia) {
      throw new Error('mic API missing');
    }
    if (state.micStream) return;
    state.micStream = await navigator.mediaDevices.getUserMedia({
      audio: { echoCancellation: false, noiseSuppression: false, autoGainControl: false }
    });
    const Ctx = window.AudioContext || window.webkitAudioContext;
    state.micCtx = new Ctx();
    const src = state.micCtx.createMediaStreamSource(state.micStream);
    state.micProc = state.micCtx.createScriptProcessor(1024, 1, 1);
    const muteSink = state.micCtx.createGain(); muteSink.gain.value = 0;
    state.micProc.onaudioprocess = (e) => {
      if (!state.txActive || !state.ws || state.ws.readyState !== WebSocket.OPEN) return;
      const inp = e.inputBuffer.getChannelData(0);
      const inRate = state.micCtx.sampleRate || 48000;
      const step = inRate / state.sampleRate;
      const outLen = Math.max(1, Math.floor(inp.length / step));
      const out = new Uint8Array(outLen);
      let s = 0, peak = 0;
      for (let i = 0; i < outLen; i++) {
        const idx = Math.min(inp.length - 1, Math.floor(s));
        const v = inp[idx] || 0;
        out[i] = linearToMulaw(v);
        const a = Math.abs(v); if (a > peak) peak = a;
        s += step;
      }
      state.txLevel = Math.max(state.txLevel * 0.85, peak);
      state.ws.send(out.buffer);
    };
    src.connect(state.micProc);
    state.micProc.connect(muteSink);
    muteSink.connect(state.micCtx.destination);
    if (state.micCtx.state === 'suspended') await state.micCtx.resume();
  }

  async function startTx() {
    if (!state.monitoring) await startMonitor();
    try { await ensureMicPath(); }
    catch (e) { setStatus('mic denied', 'err'); return; }
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
      state.ws.send('tx_start');
      state.txActive = true;
      $('#vpPtt').classList.add('live');
      setStatus('TX', 'err'); // red looks right for "live mic on air"
    }
  }

  function stopTx(localOnly = false) {
    if (!localOnly && state.ws && state.ws.readyState === WebSocket.OPEN) {
      try { state.ws.send('tx_stop'); } catch (_) {}
    }
    state.txActive = false;
    $('#vpPtt').classList.remove('live');
    if (state.monitoring) setStatus('● live', 'ok');
  }

  function toggleMute() {
    state.muted = !state.muted;
    $('#vpMute').textContent = state.muted ? '🔈' : '🔇';
    setVolumePercent(parseInt($('#vpVol').value, 10) || 80);
  }

  // 10 Hz UI tick: meters + queue depth
  setInterval(() => {
    const rxBar = $('#vpRxBar'), txBar = $('#vpTxBar'), q = $('#vpQueue');
    if (rxBar) rxBar.style.width = Math.min(100, Math.round(state.rxLevel * 100)) + '%';
    if (txBar) txBar.style.width = Math.min(100, Math.round(state.txLevel * 100)) + '%';
    if (q) {
      const ms = state.sampleRate ? Math.round((state.queuedSamples * 1000) / state.sampleRate) : 0;
      q.textContent = ms + ' ms';
    }
    state.rxLevel *= 0.85;
    state.txLevel *= 0.85;
  }, 100);

  return {
    startMonitor, stopMonitor, startTx, stopTx, toggleMute, setVolumePercent,
    state,
  };
})();
window.VoiceFM = VoiceFM;

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
  $('#stRfEn').checked    = !!radio.rf_en;
  $('#stModem').value     = String(radio.modem ?? 0);
  $('#stFx25').value      = String(radio.fx25_mode ?? 0);
  $('#stPreamble').value  = radio.preamble ?? 3;
  $('#stTxSlot').value    = radio.tx_timeslot ?? 2000;
  $('#stAudioLpf').checked= !!radio.audio_lpf;
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
    sql_level:   $('#stSql').value,
    rf_power:    $('#stPwr').value,
    tone_rx:     $('#stToneRx').value,
    tone_tx:     $('#stToneTx').value,
    rf_en:       $('#stRfEn').checked ? '1' : '0',
    modem_type:  $('#stModem').value,
    fx25_mode:   $('#stFx25').value,
    preamble:    $('#stPreamble').value,
    tx_timeslot: $('#stTxSlot').value,
    audio_lpf:   $('#stAudioLpf').checked ? '1' : '0',
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

// ---------- Filter wiring -------------------------------------------------
const filterIn   = $('#filterInput');
const filterMode = $('#filterMode');
const filterBar  = $('#filterBar');
const filterClr  = $('#filterClear');
function applyFilter() {
  filterState.text = (filterIn.value || '').trim().toUpperCase();
  filterState.mode = filterMode.value;
  filterBar.classList.toggle('active', !!filterState.text);
  applyFilterToFeed();
}
filterIn.addEventListener('input', applyFilter);
filterMode.addEventListener('change', applyFilter);
filterClr.addEventListener('click', () => { filterIn.value = ''; applyFilter(); filterIn.focus(); });

// ---------- Voice panel wiring -------------------------------------------
const voicePanel = $('#voicePanel');
$('#voiceBtn').addEventListener('click', () => {
  const wasHidden = voicePanel.hidden;
  voicePanel.hidden = !wasHidden;
  $('#voiceBtn').classList.toggle('on', !voicePanel.hidden);
});
$('#vpToggle').addEventListener('click', () => {
  if (VoiceFM.state.monitoring) VoiceFM.stopMonitor();
  else VoiceFM.startMonitor();
});
$('#vpVol').addEventListener('input', (e) => VoiceFM.setVolumePercent(+e.target.value));
$('#vpMute').addEventListener('click', () => VoiceFM.toggleMute());

// Hold-to-talk PTT (touch + mouse + keyboard space)
const ptt = $('#vpPtt');
const pttDown = (e) => { e.preventDefault(); VoiceFM.startTx(); };
const pttUp   = (e) => { e.preventDefault(); VoiceFM.stopTx();  };
['mousedown', 'touchstart'].forEach(ev => ptt.addEventListener(ev, pttDown, { passive: false }));
['mouseup', 'mouseleave', 'touchend', 'touchcancel'].forEach(ev => ptt.addEventListener(ev, pttUp, { passive: false }));
window.addEventListener('keydown', (e) => {
  // Space-to-talk only when the voice panel is visible AND no text input is focused.
  if (e.code !== 'Space' || voicePanel.hidden || e.repeat) return;
  const tag = (document.activeElement?.tagName || '').toLowerCase();
  if (tag === 'input' || tag === 'textarea') return;
  e.preventDefault();
  VoiceFM.startTx();
});
window.addEventListener('keyup', (e) => {
  if (e.code !== 'Space' || voicePanel.hidden) return;
  const tag = (document.activeElement?.tagName || '').toLowerCase();
  if (tag === 'input' || tag === 'textarea') return;
  e.preventDefault();
  VoiceFM.stopTx();
});
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
  await syncBrowserTime();
  await Promise.all([loadMe(), loadRadio()]);
  await hydrate();
  connectStream();
  // /api/me changes slowly, /api/radio carries the live audio-level meter so
  // refresh it more often.
  setInterval(loadMe,    30_000);
  setInterval(loadRadio,  3_000);
})();

})();
