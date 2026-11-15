const $ = (id) => document.getElementById(id);
const video = $("backgroundVideo");
const youtubeFrame = $("youtubeBackground");

let socket = null;
let deviceMode = "demo";
let wsUrl = "";
let bridgeWsPath = "";
let wsConnected = false;
let hadLiveEsp = false;
let demoTimer = null;
let rideClockTimer = null;
let motionFrame = null;

const MIN_CYCLING_TORQUE = 0;
const MAX_CYCLING_TORQUE = 85;
const STEP_CYCLING_TORQUE = 2;
let targetTorque = 14;
let prescription = null;
let clinicalIntake = null;

let paused = false;
let ended = false;
let saving = false;
let saved = false;

let activeMs = 0;
let lastTickAt = 0;
let simulatedCycles = 0;
let torqueSum = 0;
let torqueSamples = 0;

let rpm = 78, speed = 24.5, torque = 112;
let visual = { rpm: 78, speed: 24.5, torque: 112 };
let animationFrom = { ...visual };
let animationTo = { ...visual };
let animationStarted = 0;
const animationDuration = 360;

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
const easeOutCubic = (t) => 1 - Math.pow(1 - t, 3);

function buildTicks(groupId, cx, cy, radius, count, innerLength, outerLength) {
  const group = $(groupId);
  const ns = "http://www.w3.org/2000/svg";
  for (let index = 0; index <= count; index += 1) {
    const angle = (135 + (270 * index / count)) * Math.PI / 180;
    const x1 = cx + Math.cos(angle) * (radius - innerLength);
    const y1 = cy + Math.sin(angle) * (radius - innerLength);
    const x2 = cx + Math.cos(angle) * (radius + outerLength);
    const y2 = cy + Math.sin(angle) * (radius + outerLength);
    const line = document.createElementNS(ns, "line");
    line.setAttribute("x1", x1.toFixed(2));
    line.setAttribute("y1", y1.toFixed(2));
    line.setAttribute("x2", x2.toFixed(2));
    line.setAttribute("y2", y2.toFixed(2));
    group.appendChild(line);
  }
}

buildTicks("rpmTicks", 60, 50, 44, 34, 2.5, 1.2);
buildTicks("torqueTicks", 60, 50, 44, 34, 2.5, 1.2);
buildTicks("speedTicks", 110, 96, 86, 42, 3.2, 1.3);

function formatDuration(ms) {
  const totalSec = Math.max(0, Math.floor(ms / 1000));
  const mins = String(Math.floor(totalSec / 60)).padStart(2, "0");
  const secs = String(totalSec % 60).padStart(2, "0");
  return `${mins}:${secs}`;
}

function paintMetrics(values) {
  $("rpmValue").textContent = Math.round(values.rpm);
  $("speedValue").textContent = values.speed.toFixed(1);
  $("torqueValue").textContent = Math.round(values.torque);

  $("rpmProgress").style.strokeDashoffset = 1000 * (1 - clamp(values.rpm / 140, 0, 1));
  $("speedProgress").style.strokeDashoffset = 1000 * (1 - clamp(values.speed / 40, 0, 1));
  $("torqueProgress").style.strokeDashoffset = 1000 * (1 - clamp(values.torque / 180, 0, 1));
}

function animateMetrics(now) {
  const progress = clamp((now - animationStarted) / animationDuration, 0, 1);
  const eased = easeOutCubic(progress);
  visual = {
    rpm: animationFrom.rpm + (animationTo.rpm - animationFrom.rpm) * eased,
    speed: animationFrom.speed + (animationTo.speed - animationFrom.speed) * eased,
    torque: animationFrom.torque + (animationTo.torque - animationFrom.torque) * eased
  };
  paintMetrics(visual);

  if (progress < 1) {
    motionFrame = requestAnimationFrame(animateMetrics);
  } else {
    visual = { ...animationTo };
    paintMetrics(visual);
    motionFrame = null;
  }
}

function setMetricTargets(next) {
  animationFrom = { ...visual };
  animationTo = {
    rpm: clamp(Number(next.rpm), 0, 140),
    speed: clamp(Number(next.speed), 0, 40),
    torque: clamp(Number(next.torque), 0, 180)
  };
  animationStarted = performance.now();
  if (motionFrame === null) motionFrame = requestAnimationFrame(animateMetrics);
}

function updateRideStatusUI() {
  const pill = $("rideStatusPill");
  const pauseBtn = $("pauseResumeBtn");
  const note = $("espConnectionNote");

  pill.classList.remove("is-paused", "is-live");
  pauseBtn.classList.toggle("is-paused", paused);

  if (ended) {
    pill.textContent = "Session Ended";
    pauseBtn.disabled = true;
  } else if (paused) {
    pill.textContent = "Paused";
    pill.classList.add("is-paused");
    pauseBtn.textContent = "Resume Session";
    pauseBtn.disabled = false;
  } else if (wsConnected) {
    pill.textContent = "ESP32 Live";
    pill.classList.add("is-live");
    pauseBtn.textContent = "Pause Session";
    pauseBtn.disabled = false;
  } else {
    pill.textContent = deviceMode === "demo" ? "Demo Mode" : "Connected Preview";
    pauseBtn.textContent = "Pause Session";
    pauseBtn.disabled = false;
  }

  $("rideTimerText").textContent = formatDuration(activeMs);
  $("rideRepsText").textContent = `${Math.floor(simulatedCycles)} cycles`;

  if (wsConnected) {
    note.textContent = `Connected to ESP32 telemetry (${wsUrl}).`;
  } else if (deviceMode === "connected" && wsUrl) {
    note.textContent = `Awaiting ESP32 stream at ${wsUrl} — showing preview simulation.`;
  } else {
    note.textContent = deviceMode === "demo"
      ? "Demo mode active — cycling metrics & cycles are simulated locally."
      : "Connected preview active — click End Session to save into Track Progress.";
  }
  updateCyclingTorqueUI();
}

function updateFromESP(data) {
  if (!data || typeof data !== "object" || paused || ended) return;
  const rawRpm = data.rpm ?? data.pedal_rpm ?? data.cadence;
  const rawSpeed = data.speed ?? data.speed_kmh;
  const rawTorque = data.torque ?? data.torque_nm ?? data.eddy_brake_torque ?? data.user_pedal_torque;
  const rawReps = data.reps ?? data.pedal_cycles ?? data.cycles ?? data.completed_reps;

  const hasRpm = rawRpm !== undefined && Number.isFinite(Number(rawRpm));
  const hasSpeed = rawSpeed !== undefined && Number.isFinite(Number(rawSpeed));
  const hasTorque = rawTorque !== undefined && Number.isFinite(Number(rawTorque));
  if (!hasRpm && !hasSpeed && !hasTorque && rawReps === undefined) return;

  const nextRpm = hasRpm ? Number(rawRpm) : animationTo.rpm;
  const next = {
    rpm: nextRpm,
    speed: hasSpeed ? Number(rawSpeed) : animationTo.speed,
    torque: hasTorque ? Number(rawTorque) : animationTo.torque
  };
  rpm = next.rpm;
  speed = next.speed;
  torque = next.torque;
  torqueSum += next.torque;
  torqueSamples += 1;
  if (rawReps !== undefined && Number.isFinite(Number(rawReps))) {
    simulatedCycles = Math.max(simulatedCycles, Number(rawReps));
  }
  setMetricTargets(next);
}

function startDemo() {
  stopDemo();
  if (paused || ended || wsConnected) return;
  demoTimer = setInterval(() => {
    if (paused || ended || wsConnected) return;
    rpm = clamp(rpm + (Math.random() - 0.48) * 5, 45, 125);
    speed = clamp(speed + (Math.random() - 0.5) * 1.1, 5, 40);
    torque = clamp(torque + (Math.random() - 0.5) * 7, 35, 175);
    torqueSum += torque;
    torqueSamples += 1;
    setMetricTargets({ rpm, speed, torque });
  }, 250);
}

function stopDemo() {
  if (demoTimer !== null) clearInterval(demoTimer);
  demoTimer = null;
}

function startRideClock() {
  stopRideClock();
  lastTickAt = performance.now();
  rideClockTimer = setInterval(() => {
    const now = performance.now();
    const dt = now - lastTickAt;
    lastTickAt = now;
    if (!paused && !ended) {
      activeMs += dt;
      if (!wsConnected) {
        // Count 1 cycling rep/cycle roughly every 3.5 seconds weighted by cadence when ESP32 stream is absent.
        const cadenceFactor = clamp(rpm / 75, 0.4, 1.8);
        simulatedCycles += (dt / 3500) * cadenceFactor;
      }
      $("rideTimerText").textContent = formatDuration(activeMs);
      $("rideRepsText").textContent = `${Math.floor(simulatedCycles)} cycles`;
    }
  }, 500);
}

function stopRideClock() {
  if (rideClockTimer !== null) clearInterval(rideClockTimer);
  rideClockTimer = null;
}

function controlBackgroundPlayback(play) {
  if (video && video.src) {
    if (play) video.play().catch(() => {});
    else video.pause();
  }
  if (youtubeFrame?.contentWindow) {
    youtubeFrame.contentWindow.postMessage(
      JSON.stringify({ event: "command", func: play ? "playVideo" : "pauseVideo", args: [] }),
      "https://www.youtube.com"
    );
  }
}

function sendEspCommand(payload) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(payload));
    return { sentLive: true };
  }
  return { sentLive: false };
}

function buildCyclingStartPayload() {
  const cyc = prescription?.cycling;
  return {
    type: "start_session",
    mode: "cycling",
    op_mode: 7,
    role: "resistance",
    torque_nm: targetTorque,
    target_torque_ext: targetTorque,
    target_reps: cyc?.suggestedCycles || 25,
    pathology: clinicalIntake?.pathology ?? null,
    d_offset_m: Number(((clinicalIntake?.dOffsetCm ?? 9.0) / 100).toFixed(3)),
    L_leg_m: Number(((clinicalIntake?.lLegCm ?? 30.0) / 100).toFixed(3))
  };
}

function updateCyclingTorqueUI() {
  const valEl = $("cyclingTargetTorqueValue");
  const minusBtn = $("cyclingTorqueMinusBtn");
  const plusBtn = $("cyclingTorquePlusBtn");
  const statusEl = $("cyclingCmdStatus");
  if (valEl) valEl.textContent = `${targetTorque} Nm`;
  if (minusBtn) minusBtn.disabled = targetTorque <= MIN_CYCLING_TORQUE || ended;
  if (plusBtn) plusBtn.disabled = targetTorque >= MAX_CYCLING_TORQUE || ended;
  if (statusEl && !statusEl.dataset.lockedMessage) {
    if (ended) {
      statusEl.textContent = "Session ended";
    } else if (wsConnected) {
      statusEl.textContent = `ESP32 Live · Target ${targetTorque} Nm`;
    } else if (deviceMode === "connected" && wsUrl) {
      statusEl.textContent = `Connecting to ESP32 · Target ${targetTorque} Nm`;
    } else {
      statusEl.textContent = deviceMode === "demo" ? "Demo mode · Ready for ESP" : "Connected preview · Target set";
    }
  }
}

function adjustCyclingTorque(delta) {
  if (ended) return;
  const next = clamp(targetTorque + delta, MIN_CYCLING_TORQUE, MAX_CYCLING_TORQUE);
  if (next === targetTorque) return;
  targetTorque = next;
  const statusEl = $("cyclingCmdStatus");
  if (statusEl) delete statusEl.dataset.lockedMessage;
  const { sentLive } = sendEspCommand({
    type: "set_torque",
    mode: "cycling",
    op_mode: 7,
    role: "resistance",
    torque_nm: next,
    target_torque_ext: next,
    delta
  });
  if (statusEl) {
    statusEl.dataset.lockedMessage = "1";
    statusEl.textContent = sentLive
      ? `Sent to ESP: ${next} Nm`
      : `Target set to ${next} Nm (offline)`;
  }
  updateCyclingTorqueUI();
}

const SCENE_VIDEOS = {
  forest: "https://www.youtube.com/watch?v=MWyBgudQqsI&pp=ygUYZm9yZXN0IGN5Y2xpbmcgcG92IHZpZGVv",
  beach: "https://www.youtube.com/watch?v=YOUR_BEACH_ID",
  nightCity: "https://www.youtube.com/watch?v=XX8km2vJbkQ&pp=ygUTdG9reW8gbmlnaHQgY3ljbGluZw%3D%3D",
  mountains: "https://www.youtube.com/shorts/1euzUN4eSkE"
};

function getYouTubeID(url) {
  try {
    const parsed = new URL(url);
    if (parsed.hostname.includes("youtu.be")) return parsed.pathname.substring(1);
    if (parsed.hostname.includes("youtube.com")) {
      if (parsed.pathname === "/watch") return parsed.searchParams.get("v");
      if (parsed.pathname.startsWith("/embed/") || parsed.pathname.startsWith("/shorts/")) {
        return parsed.pathname.split("/")[2];
      }
    }
  } catch (error) {
    console.error("Invalid YouTube URL", error);
  }
  return null;
}

function changeScene(scene) {
  const url = SCENE_VIDEOS[scene];
  const videoID = getYouTubeID(url);
  if (!videoID || videoID.startsWith("YOUR_")) return;

  video.pause();
  video.removeAttribute("src");
  video.load();

  youtubeFrame.src =
    `https://www.youtube.com/embed/${videoID}` +
    `?enablejsapi=1&autoplay=1&mute=1&controls=0&loop=1` +
    `&playlist=${videoID}&playsinline=1&rel=0`;
}

function connectEspSocket() {
  if (deviceMode !== "connected" || !wsUrl) return;
  const endpoint = bridgeWsPath
    ? `${window.location.protocol === "https:" ? "wss" : "ws"}://${window.location.host}${bridgeWsPath}`
    : wsUrl;
  const usingBridge = Boolean(bridgeWsPath);
  if (!endpoint || !/^wss?:\/\//.test(endpoint)) return;

  try {
    socket = new WebSocket(endpoint);
    socket.addEventListener("open", () => {
      if (!usingBridge) {
        wsConnected = true;
        stopDemo();
        updateRideStatusUI();
        sendEspCommand(buildCyclingStartPayload());
      } else {
        updateRideStatusUI();
      }
    });
    socket.addEventListener("message", (event) => {
      try {
        const packet = JSON.parse(event.data);
        if (!packet || typeof packet !== "object") return;

        if (packet.type === "esp_status") {
          wsConnected = Boolean(packet.connected);
          const statusEl = $("cyclingCmdStatus");
          if (statusEl) delete statusEl.dataset.lockedMessage;
          if (wsConnected) {
            stopDemo();
            if (!paused && !ended) sendEspCommand(buildCyclingStartPayload());
            if (paused) sendEspCommand({ type: "pause_session" });
          } else if (!paused && !ended) {
            startDemo();
          }
          updateRideStatusUI();
          return;
        }

        if (packet.type === "esp_ack") {
          if (packet.forwarded === false && $("cyclingCmdStatus")) {
            $("cyclingCmdStatus").dataset.lockedMessage = "1";
            $("cyclingCmdStatus").textContent = packet.message || "ESP32 offline — target saved locally";
          }
          return;
        }

        hadLiveEsp = true;
        if (!wsConnected) {
          wsConnected = true;
          stopDemo();
          const statusEl = $("cyclingCmdStatus");
          if (statusEl) delete statusEl.dataset.lockedMessage;
          updateRideStatusUI();
        }
        updateFromESP(packet);
      } catch (error) {
        console.warn("Invalid ESP32 data packet", error);
      }
    });
    socket.addEventListener("close", () => {
      wsConnected = false;
      updateRideStatusUI();
      if (!paused && !ended) startDemo();
    });
    socket.addEventListener("error", () => {
      wsConnected = false;
      updateRideStatusUI();
    });
  } catch {
    wsConnected = false;
  }
}

async function persistCyclingSession() {
  if (saving || saved) return;
  saving = true;
  const statusEl = $("summarySaveStatus");
  const retryBtn = $("retrySaveBtn");
  statusEl.className = "summary-status";
  statusEl.textContent = "Saving session to Track Progress…";
  retryBtn.hidden = true;

  const durationSeconds = Math.max(0, Math.round(activeMs / 1000));
  const reps = clamp(Math.round(simulatedCycles), 1, 99);
  const avgTorque = Math.round((torqueSamples ? torqueSum / torqueSamples : torque) * 10) / 10;

  $("summaryDuration").textContent = formatDuration(durationSeconds * 1000);
  $("summaryReps").textContent = String(reps);
  $("summaryTorque").textContent = `${avgTorque} Nm`;

  try {
    const response = await fetch("/api/progress", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      credentials: "same-origin",
      body: JSON.stringify({
        type: "Cycling",
        mode: "Cycling",
        torqueNm: avgTorque,
        reps,
        durationSeconds,
        source: (wsConnected || hadLiveEsp)
          ? "esp32"
          : (deviceMode === "demo" ? "demo" : "connected-preview")
      })
    });
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.message || "Could not save session.");
    saved = true;
    statusEl.className = "summary-status saved";
    statusEl.textContent = "Session saved to Track Progress ✓";
  } catch (error) {
    statusEl.className = "summary-status error";
    statusEl.textContent = `${error.message} Click Retry Save.`;
    retryBtn.hidden = false;
  } finally {
    saving = false;
  }
}

function togglePauseSession() {
  if (ended) return;
  paused = !paused;
  lastTickAt = performance.now();
  if (paused) {
    stopDemo();
    controlBackgroundPlayback(false);
    sendEspCommand({ type: "pause_session" });
  } else {
    controlBackgroundPlayback(true);
    sendEspCommand({ type: "resume_session" });
    if (!wsConnected) startDemo();
  }
  updateRideStatusUI();
}

function endCyclingSession() {
  if (ended) return;
  ended = true;
  paused = true;
  stopDemo();
  stopRideClock();
  controlBackgroundPlayback(false);
  sendEspCommand({ type: "stop_session" });
  updateRideStatusUI();
  $("menuPanel").classList.remove("open");
  $("sessionSummaryModal").hidden = false;
  void persistCyclingSession();
}

function restartCyclingSession() {
  ended = false;
  paused = false;
  saved = false;
  saving = false;
  hadLiveEsp = false;
  activeMs = 0;
  simulatedCycles = 0;
  torqueSum = 0;
  torqueSamples = 0;
  lastTickAt = performance.now();
  const statusEl = $("cyclingCmdStatus");
  if (statusEl) delete statusEl.dataset.lockedMessage;
  $("sessionSummaryModal").hidden = true;
  controlBackgroundPlayback(true);
  if (wsConnected) sendEspCommand(buildCyclingStartPayload());
  else startDemo();
  startRideClock();
  updateRideStatusUI();
}

function updateClock() {
  $("clock").textContent = new Intl.DateTimeFormat([], {
    hour: "2-digit", minute: "2-digit"
  }).format(new Date());
}

function renderCyclingPrescription(rx) {
  const banner = $("cyclingRecBanner");
  const menuNote = $("cyclingRxNote");
  if (!banner) return;

  if (!rx || !rx.cycling) {
    banner.hidden = true;
    if (menuNote) menuNote.hidden = true;
    return;
  }

  const cyc = rx.cycling;
  banner.textContent = "";
  banner.hidden = false;
  banner.classList.toggle("is-caution", !cyc.allowed);

  const textWrap = document.createElement("div");
  textWrap.className = "cycling-rx-text";

  const badge = document.createElement("span");
  badge.className = "cycling-rx-badge";
  badge.textContent = cyc.allowed ? `${rx.protocolId} · Suggested` : `${rx.protocolId} · Caution`;

  const msg = document.createElement("span");
  msg.textContent = cyc.allowed
    ? `${cyc.modeName} · Target ${cyc.targetTorqueNm} Nm (${cyc.torqueRangeNm}) · ${cyc.cadenceRangeRpm} RPM · ${cyc.powerRangeW} · ${cyc.durationMin} min`
    : `${cyc.priorityLabel} — ${cyc.note}`;

  textWrap.append(badge, msg);

  const action = document.createElement("a");
  action.className = "cycling-rx-action";
  action.href = cyc.allowed ? "/intake" : "/device/flexion/";
  action.textContent = cyc.allowed ? "Update Intake →" : "Switch to Flexion →";

  banner.append(textWrap, action);

  if (menuNote) {
    menuNote.hidden = false;
    menuNote.textContent = cyc.allowed
      ? `Rx: ${cyc.modeName}, ${cyc.targetTorqueNm} Nm, ${cyc.cadenceRangeRpm} RPM, ${cyc.durationMin} min.`
      : `Rx Hold: ${cyc.priorityLabel}. Use Flexion & Extension until ROM >= 100°.`;
  }
}

async function initSession() {
  try {
    const res = await fetch("/api/device/config", {
      credentials: "same-origin",
      cache: "no-store"
    });
    if (res.ok) {
      const data = await res.json();
      if (data.ok) {
        deviceMode = data.deviceMode === "connected" ? "connected" : "demo";
        wsUrl = data.wsUrl || "";
        bridgeWsPath = data.bridgeWsPath || "";
        prescription = data.prescription || null;
        clinicalIntake = data.clinicalIntake || null;
        const rxTorque = Number(prescription?.cycling?.targetTorqueNm);
        if (Number.isFinite(rxTorque) && rxTorque > 0) {
          targetTorque = clamp(Math.round(rxTorque), MIN_CYCLING_TORQUE, MAX_CYCLING_TORQUE);
        }
        renderCyclingPrescription(prescription);
      }
    }
  } catch {}
  updateRideStatusUI();
  connectEspSocket();
  if (!wsConnected) startDemo();
}

updateClock();
setInterval(updateClock, 15000);
paintMetrics(visual);
changeScene("forest");
startRideClock();
void initSession();

$("menuButton").addEventListener("click", () => $("menuPanel").classList.toggle("open"));
$("pauseResumeBtn").addEventListener("click", togglePauseSession);
$("endSessionBtn").addEventListener("click", endCyclingSession);
$("retrySaveBtn").addEventListener("click", () => void persistCyclingSession());
$("restartRideBtn").addEventListener("click", restartCyclingSession);
$("cyclingTorqueMinusBtn")?.addEventListener("click", () => adjustCyclingTorque(-STEP_CYCLING_TORQUE));
$("cyclingTorquePlusBtn")?.addEventListener("click", () => adjustCyclingTorque(STEP_CYCLING_TORQUE));

$("fullscreenButton").addEventListener("click", async () => {
  try {
    if (!document.fullscreenElement) await $("dashboard").requestFullscreen();
    else await document.exitFullscreen();
  } catch (error) {
    console.warn("Fullscreen unavailable", error);
  }
});

$("switchSessionButton").addEventListener("click", () => window.location.assign("/device/"));
$("backDashboardButton").addEventListener("click", () => window.location.assign("/dashboard"));

document.querySelectorAll(".scene-option").forEach((button) => {
  button.addEventListener("click", () => {
    document.querySelectorAll(".scene-option").forEach((item) => {
      item.classList.remove("active");
      item.setAttribute("aria-pressed", "false");
    });
    button.classList.add("active");
    button.setAttribute("aria-pressed", "true");
    $("sceneName").textContent = button.dataset.label;
    changeScene(button.dataset.scene);
  });
});

window.doulaCycling = { updateFromESP };
