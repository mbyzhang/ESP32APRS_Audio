(() => {
  const ctcss = [0, 67, 71.9, 74.4, 77, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8, 97.4, 100, 103.5, 107.2, 110.9, 114.8, 118.8, 123, 127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2, 192.8, 203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 250.3];

  const state = {
    activeTab: "home",
    selectedContact: "",
    contacts: [],
    audioRunning: false,
    audioMuted: false,
    audioLevel: 0,
    wsAudio: null,
    audioCtx: null,
    gainNode: null,
    procNode: null,
    queue: [],
    queueOffset: 0,
    queuedSamples: 0,
    sampleRate: 8000,
    currentSample: 0,
    resampleAcc: 0,
    micStream: null,
    micAnalyser: null,
    micData: null,
    micRaf: 0,
    pttDown: false
  };

  const $ = (id) => document.getElementById(id);

  async function api(path, opts = {}) {
    const resp = await fetch(path, {
      cache: "no-store",
      ...opts
    });
    const isJson = (resp.headers.get("content-type") || "").includes("application/json");
    const data = isJson ? await resp.json() : await resp.text();
    if (!resp.ok) {
      const msg = (data && data.message) ? data.message : `HTTP ${resp.status}`;
      throw new Error(msg);
    }
    return data;
  }

  function formBody(data) {
    return new URLSearchParams(data).toString();
  }

  function setStatus(line) {
    $("statusLine").textContent = line;
  }

  function fmtTs(ts) {
    if (!ts) return "-";
    const d = new Date(ts * 1000);
    return d.toLocaleString();
  }

  function escapeHtml(s) {
    return String(s || "")
      .replaceAll("&", "&amp;")
      .replaceAll("<", "&lt;")
      .replaceAll(">", "&gt;")
      .replaceAll('"', "&quot;");
  }

  async function refreshStatus() {
    try {
      const st = await api("/api/status");
      const sta = st.sta_connected ? `STA ${st.sta_ip}` : "STA disconnected";
      const ap = `AP ${st.ap_ip}`;
      setStatus(`${sta} | ${ap} | ${st.ap_ssid}`);
    } catch (err) {
      setStatus(`Status error: ${err.message}`);
    }
  }

  function renderContacts() {
    const root = $("contactsList");
    root.innerHTML = "";
    if (!state.contacts.length) {
      root.innerHTML = '<div class="contact-item">No recent stations</div>';
      return;
    }
    for (const c of state.contacts) {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = `contact-item ${state.selectedContact === c.call ? "active" : ""}`;
      btn.innerHTML = `<strong>${escapeHtml(c.call)}</strong><div class="muted">${fmtTs(c.last_active)}</div>`;
      btn.onclick = () => {
        state.selectedContact = c.call;
        $("chatTitle").textContent = c.call;
        $("sendTo").value = c.call;
        renderContacts();
        refreshMessages();
      };
      root.appendChild(btn);
    }
  }

  async function refreshContacts() {
    try {
      const data = await api("/api/contacts");
      state.contacts = Array.isArray(data.contacts) ? data.contacts : [];
      if (!state.selectedContact && state.contacts.length) {
        state.selectedContact = state.contacts[0].call;
        $("chatTitle").textContent = state.selectedContact;
        $("sendTo").value = state.selectedContact;
      }
      renderContacts();
      await refreshMessages();
    } catch (err) {
      console.error(err);
    }
  }

  async function refreshMessages() {
    const root = $("messagesList");
    const c = state.selectedContact;
    if (!c) {
      root.innerHTML = '<div class="muted">Choose a station to view chat.</div>';
      return;
    }
    try {
      const data = await api(`/api/messages?contact=${encodeURIComponent(c)}`);
      const list = Array.isArray(data.messages) ? data.messages : [];
      root.innerHTML = "";
      for (const m of list) {
        const el = document.createElement("div");
        const dir = m.dir === "tx" ? "tx" : "rx";
        const ack = m.ack > 0 ? `retry ${m.ack}` : (m.ack === -2 ? "acked" : (m.ack === -1 ? "rx" : "pending"));
        el.className = `msg ${dir}`;
        el.innerHTML = `<div>${escapeHtml(m.text)}</div><div class="msg-meta">${fmtTs(m.ts)} | ${escapeHtml(ack)} | #${m.msg_id}</div>`;
        root.appendChild(el);
      }
      root.scrollTop = root.scrollHeight;
    } catch (err) {
      root.innerHTML = `<div class="muted">Message load failed: ${escapeHtml(err.message)}</div>`;
    }
  }

  async function sendMessage(ev) {
    ev.preventDefault();
    const to = $("sendTo").value.trim().toUpperCase();
    const text = $("sendText").value.trim();
    if (!to || !text) return;
    try {
      await api("/api/message/send", {
        method: "POST",
        headers: { "Content-Type": "application/x-www-form-urlencoded;charset=UTF-8" },
        body: formBody({ to, text })
      });
      $("sendText").value = "";
      state.selectedContact = to;
      $("chatTitle").textContent = to;
      await refreshContacts();
    } catch (err) {
      alert(`Send failed: ${err.message}`);
    }
  }

  function popQueueSample() {
    if (!state.queue.length) return 0;
    const chunk = state.queue[0];
    const sample = chunk[state.queueOffset++];
    state.queuedSamples--;
    if (state.queueOffset >= chunk.length) {
      state.queue.shift();
      state.queueOffset = 0;
    }
    return sample;
  }

  function clearAudioQueue() {
    state.queue = [];
    state.queueOffset = 0;
    state.queuedSamples = 0;
    state.currentSample = 0;
    state.resampleAcc = 0;
  }

  function mulawToLinear(uVal) {
    uVal = (~uVal) & 0xff;
    const sign = uVal & 0x80;
    const exponent = (uVal >> 4) & 0x07;
    const mantissa = uVal & 0x0f;
    let sample = ((mantissa << 3) + 0x84) << exponent;
    sample -= 0x84;
    return sign ? -sample : sample;
  }

  function setAudioGain() {
    const volume = Number($("audioVolume").value || 0) / 100;
    if (state.gainNode) {
      state.gainNode.gain.value = state.audioMuted ? 0 : volume;
    }
  }

  function ensureAudioContext() {
    if (state.audioCtx) return;
    state.audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    state.gainNode = state.audioCtx.createGain();
    state.procNode = state.audioCtx.createScriptProcessor(1024, 1, 1);
    state.procNode.onaudioprocess = (e) => {
      const out = e.outputBuffer.getChannelData(0);
      const outRate = state.audioCtx.sampleRate;
      for (let i = 0; i < out.length; i++) {
        state.resampleAcc += state.sampleRate;
        while (state.resampleAcc >= outRate) {
          state.currentSample = popQueueSample();
          state.resampleAcc -= outRate;
        }
        out[i] = state.currentSample;
      }
    };
    state.procNode.connect(state.gainNode);
    state.gainNode.connect(state.audioCtx.destination);
    setAudioGain();
  }

  async function tune(action, freq) {
    const q = new URLSearchParams({ action });
    if (typeof freq === "number" && Number.isFinite(freq)) {
      q.set("freq", freq.toFixed(4));
    }
    return api(`/audio_tune?${q.toString()}`);
  }

  async function updateTuneStatus() {
    try {
      const st = await tune("status");
      const txt = st.active
        ? `Tuned ${Number(st.rx).toFixed(4)} MHz (auto-return ${st.timeoutSec}s idle)`
        : `APRS lock ${Number(st.aprs).toFixed(4)} MHz`;
      $("tuneStatus").textContent = txt;
      if (document.activeElement !== $("tuneFreq")) {
        $("tuneFreq").value = Number(st.rx).toFixed(4);
      }
    } catch (_) {}
  }

  async function startAudio() {
    ensureAudioContext();
    await state.audioCtx.resume();
    clearAudioQueue();

    const wsProto = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${wsProto}://${location.host}/ws_audio`);
    ws.binaryType = "arraybuffer";
    state.wsAudio = ws;

    ws.onopen = () => {
      state.audioRunning = true;
      $("audioToggle").textContent = "Stop Listening";
      $("audioState").textContent = "Connected";
      tune("touch").catch(() => {});
    };

    ws.onclose = () => {
      stopAudio();
    };

    ws.onerror = () => {
      $("audioState").textContent = "Socket error";
    };

    ws.onmessage = (event) => {
      if (typeof event.data === "string") return;
      const ulaw = new Uint8Array(event.data);
      const pcm = new Float32Array(ulaw.length);
      let peak = 0;
      for (let i = 0; i < ulaw.length; i++) {
        const s = mulawToLinear(ulaw[i]) / 32768;
        pcm[i] = s;
        peak = Math.max(peak, Math.abs(s));
      }
      state.audioLevel = peak;
      state.queue.push(pcm);
      state.queuedSamples += pcm.length;
      while (state.queuedSamples > state.sampleRate * 2 && state.queue.length) {
        state.queuedSamples -= state.queue[0].length;
        state.queue.shift();
        state.queueOffset = 0;
      }
    };
  }

  function stopAudio() {
    state.audioRunning = false;
    if (state.wsAudio) {
      state.wsAudio.onopen = null;
      state.wsAudio.onmessage = null;
      state.wsAudio.onclose = null;
      state.wsAudio.onerror = null;
      state.wsAudio.close();
      state.wsAudio = null;
    }
    clearAudioQueue();
    $("audioToggle").textContent = "Start Listening";
    $("audioState").textContent = "Stopped";
  }

  async function refreshRadioStatus() {
    try {
      const r = await api("/api/radio/status");
      $("radioRx").value = Number(r.freq_rx).toFixed(4);
      $("radioTx").value = Number(r.freq_tx).toFixed(4);
      $("toneRx").value = String(r.tone_rx);
      $("toneTx").value = String(r.tone_tx);
      $("sqlLevel").value = String(r.sql_level);
      $("rfVolume").value = String(r.volume);
      $("pttState").textContent = `PTT: ${r.ptt ? "ON" : "idle"} | SQ: ${r.sq}`;
    } catch (err) {
      $("pttState").textContent = `Radio status error: ${err.message}`;
    }
  }

  async function saveRadio() {
    const body = formBody({
      freq_rx: $("radioRx").value,
      freq_tx: $("radioTx").value,
      tone_rx: $("toneRx").value,
      tone_tx: $("toneTx").value,
      sql_level: $("sqlLevel").value,
      volume: $("rfVolume").value,
      save: "1"
    });
    try {
      await api("/api/radio/set", {
        method: "POST",
        headers: { "Content-Type": "application/x-www-form-urlencoded;charset=UTF-8" },
        body
      });
      await refreshRadioStatus();
    } catch (err) {
      alert(`Radio save failed: ${err.message}`);
    }
  }

  async function setPtt(on) {
    await api("/api/radio/ptt", {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded;charset=UTF-8" },
      body: formBody({ state: on ? "1" : "0", hold_ms: "2000" })
    });
    await refreshRadioStatus();
  }

  async function refreshDiag() {
    try {
      const d = await api("/api/diag/rf");
      const rows = [
        ["PTT", d.ptt ? "ON" : "OFF"],
        ["SQ", d.sq],
        ["TX Audio", d.tx_audio_activity ? "active" : "idle"],
        ["RX Audio", `${d.rx_audio_mv} mV`],
        ["STA", d.sta_connected ? d.sta_ip : "disconnected"],
        ["AP", d.ap_ip]
      ];
      const html = rows.map(([k, v]) => `<dt>${escapeHtml(k)}</dt><dd>${escapeHtml(String(v))}</dd>`).join("");
      $("diagList").innerHTML = html;
    } catch (err) {
      $("diagList").innerHTML = `<dt>Error</dt><dd>${escapeHtml(err.message)}</dd>`;
    }
  }

  async function runSelftest() {
    try {
      const r = await api("/api/selftest");
      $("selftestSummary").textContent = r.kv4p_2_0d_pass ? "PASS" : "FAIL / unsupported";
      const checks = Array.isArray(r.checks) ? r.checks : [];
      $("selftestList").innerHTML = checks.map((c) => {
        const klass = c.ok ? "selftest-ok" : "selftest-bad";
        return `<div class="${klass}">${escapeHtml(c.name)}: ${c.current} (exp ${c.expected})</div>`;
      }).join("");
    } catch (err) {
      $("selftestSummary").textContent = `Self-test error: ${err.message}`;
    }
  }

  async function loadWebhookCfg() {
    try {
      const c = await api("/api/config");
      $("hookEnable").checked = !!c.msg_webhook_enable;
      $("hookUrl").value = c.msg_webhook_url || "";
      $("hookTimeout").value = c.msg_webhook_timeout_ms || 1500;
    } catch (err) {
      console.error(err);
    }
  }

  async function saveWebhookCfg() {
    try {
      await api("/api/config", {
        method: "POST",
        headers: { "Content-Type": "application/x-www-form-urlencoded;charset=UTF-8" },
        body: formBody({
          msg_webhook_enable: $("hookEnable").checked ? "1" : "0",
          msg_webhook_url: $("hookUrl").value.trim(),
          msg_webhook_timeout_ms: $("hookTimeout").value
        })
      });
      await loadWebhookCfg();
      alert("Webhook settings saved.");
    } catch (err) {
      alert(`Save failed: ${err.message}`);
    }
  }

  async function restoreConfig() {
    const file = $("restoreCfgFile").files[0];
    if (!file) return;
    const fd = new FormData();
    fd.append("file", file, "default.cfg");
    try {
      const resp = await fetch("/api/config/restore", { method: "POST", body: fd });
      const txt = await resp.text();
      if (!resp.ok) throw new Error(txt || `HTTP ${resp.status}`);
      alert("Config restored. Reloading status.");
      await refreshStatus();
      await refreshRadioStatus();
      await loadWebhookCfg();
    } catch (err) {
      alert(`Restore failed: ${err.message}`);
    }
  }

  async function uploadOta() {
    const file = $("otaFile").files[0];
    if (!file) return;
    const fd = new FormData();
    fd.append("firmware", file, file.name || "firmware.bin");
    try {
      const resp = await fetch("/update", { method: "POST", body: fd });
      const text = await resp.text();
      if (!resp.ok) throw new Error(text || `HTTP ${resp.status}`);
      alert("OTA upload complete. Device will reboot.");
    } catch (err) {
      alert(`OTA failed: ${err.message}`);
    }
  }

  async function enableMic() {
    if (state.micStream) return;
    try {
      state.micStream = await navigator.mediaDevices.getUserMedia({ audio: true, video: false });
      const ctx = new (window.AudioContext || window.webkitAudioContext)();
      const src = ctx.createMediaStreamSource(state.micStream);
      const analyser = ctx.createAnalyser();
      analyser.fftSize = 1024;
      src.connect(analyser);
      state.micAnalyser = analyser;
      state.micData = new Uint8Array(analyser.fftSize);

      const loop = () => {
        if (!state.micAnalyser) return;
        state.micAnalyser.getByteTimeDomainData(state.micData);
        let peak = 0;
        for (let i = 0; i < state.micData.length; i++) {
          const v = Math.abs(state.micData[i] - 128) / 128;
          if (v > peak) peak = v;
        }
        $("micLevel").style.width = `${Math.round(Math.min(1, peak * 3) * 100)}%`;
        state.micRaf = requestAnimationFrame(loop);
      };
      loop();
      $("micEnable").textContent = "Mic Enabled";
    } catch (err) {
      alert(`Mic access failed: ${err.message}`);
    }
  }

  function populateToneSelects() {
    const mk = (el) => {
      el.innerHTML = ctcss.map((v, i) => `<option value="${i}">${i === 0 ? "OFF" : `${v.toFixed(1)} Hz`}</option>`).join("");
    };
    mk($("toneRx"));
    mk($("toneTx"));
  }

  function switchTab(tab) {
    state.activeTab = tab;
    document.querySelectorAll(".tab-pane").forEach((p) => p.classList.toggle("active", p.id === `tab-${tab}`));
    document.querySelectorAll(".tabbar button").forEach((b) => b.classList.toggle("active", b.dataset.tab === tab));
  }

  function bindEvents() {
    document.querySelectorAll(".tabbar button").forEach((b) => {
      b.onclick = () => switchTab(b.dataset.tab);
    });

    $("openLegacy").onclick = () => { location.href = "/legacy"; };
    $("refreshContacts").onclick = refreshContacts;
    $("sendForm").addEventListener("submit", sendMessage);

    $("audioToggle").onclick = async () => {
      if (state.audioRunning) stopAudio();
      else await startAudio();
    };
    $("audioMute").onclick = () => {
      state.audioMuted = !state.audioMuted;
      $("audioMute").textContent = state.audioMuted ? "Unmute" : "Mute";
      setAudioGain();
    };
    $("audioVolume").oninput = setAudioGain;

    $("tuneSet").onclick = async () => {
      const freq = Number($("tuneFreq").value);
      if (!Number.isFinite(freq)) return;
      try {
        await tune("set", freq);
        await updateTuneStatus();
      } catch (err) {
        $("tuneStatus").textContent = `Tune error: ${err.message}`;
      }
    };
    $("tuneReset").onclick = async () => {
      await tune("reset");
      await updateTuneStatus();
    };

    $("saveRadio").onclick = saveRadio;

    const pttBtn = $("pttBtn");
    const down = async (e) => {
      e.preventDefault();
      if (state.pttDown) return;
      state.pttDown = true;
      try { await setPtt(true); } catch (_) {}
    };
    const up = async (e) => {
      e.preventDefault();
      if (!state.pttDown) return;
      state.pttDown = false;
      try { await setPtt(false); } catch (_) {}
    };
    ["mousedown", "touchstart", "pointerdown"].forEach((ev) => pttBtn.addEventListener(ev, down, { passive: false }));
    ["mouseup", "mouseleave", "touchend", "touchcancel", "pointerup", "pointercancel"].forEach((ev) => pttBtn.addEventListener(ev, up, { passive: false }));

    $("micEnable").onclick = enableMic;
    $("refreshDiag").onclick = refreshDiag;
    $("runSelftest").onclick = runSelftest;
    $("saveWebhook").onclick = saveWebhookCfg;
    $("backupCfg").onclick = () => { window.open("/api/config/backup", "_blank"); };
    $("restoreCfg").onclick = restoreConfig;
    $("otaUpload").onclick = uploadOta;
  }

  function startIntervals() {
    setInterval(() => {
      $("audioLevel").style.width = `${Math.round(Math.min(1, state.audioLevel) * 100)}%`;
      state.audioLevel *= 0.86;
      if (state.audioRunning && state.wsAudio && state.wsAudio.readyState === WebSocket.OPEN) {
        state.wsAudio.send("ping");
      }
    }, 120);

    setInterval(refreshStatus, 5000);
    setInterval(() => {
      if (state.activeTab === "home") refreshContacts();
    }, 7000);
    setInterval(() => {
      if (state.activeTab === "audio") {
        updateTuneStatus();
        refreshRadioStatus();
      }
    }, 5000);
    setInterval(() => {
      if (state.activeTab === "diag") refreshDiag();
    }, 4000);
  }

  async function init() {
    populateToneSelects();
    bindEvents();
    await refreshStatus();
    await refreshContacts();
    await refreshRadioStatus();
    await updateTuneStatus();
    await refreshDiag();
    await loadWebhookCfg();
    startIntervals();
  }

  init().catch((err) => {
    setStatus(`Init failed: ${err.message}`);
  });
})();
