const $ = (id) => document.getElementById(id);
const video = $("backgroundVideo");
const youtubeFrame = $("youtubeBackground");

let socket = null;
let deviceMode = "demo";
let wsUrl = "";
let wsConnected = false;
let demoTimer = null;
let rideClockTimer = null;
let motionFrame = null;

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
  } else if (wsUrl) {
    note.textContent = `ESP32 offline at ${wsUrl} — showing preview simulation.`;
  } else {
    note.textContent = deviceMode === "demo"
      ? "Demo mode active — cycling metrics & cycles are simulated locally."
      : "Connected preview active — click End Session to save into Track Progress.";
  }
}

function updateFromESP(data) {
  if (!data || typeof data !== "object" || paused || ended) return;
  const next = {
    rpm: Number.isFinite(Number(data.rpm)) ? Number(data.rpm) : animationTo.rpm,
    speed: Number.isFinite(Number(data.speed)) ? Number(data.speed) : animationTo.speed,
    torque: Number.isFinite(Number(data.torque)) ? Number(data.torque) : animationTo.torque
  };
  rpm = next.rpm;
  speed = next.speed;
  torque = next.torque;
  torqueSum += next.torque;
  torqueSamples += 1;
  if (Number.isFinite(Number(data.reps))) {
    simulatedCycles = Math.max(simulatedCycles, Number(data.reps));
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
      // Count 1 cycling rep/cycle roughly every 3.5 seconds weighted by cadence.
      const cadenceFactor = clamp(rpm / 75, 0.4, 1.8);
      simulatedCycles += (dt / 3500) * cadenceFactor;
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
  }
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
  if (!wsUrl || !/^wss?:\/\//.test(wsUrl)) return;
  try {
    socket = new WebSocket(wsUrl);
    socket.addEventListener("open", () => {
      wsConnected = true;
      stopDemo();
      updateRideStatusUI();
    });
    socket.addEventListener("message", (event) => {
      try { updateFromESP(JSON.parse(event.data)); }
      catch (error) { console.warn("Invalid ESP32 data packet", error); }
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
        source: deviceMode === "demo" ? "demo" : "connected-preview"
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
  activeMs = 0;
  simulatedCycles = 0;
  torqueSum = 0;
  torqueSamples = 0;
  lastTickAt = performance.now();
  $("sessionSummaryModal").hidden = true;
  controlBackgroundPlayback(true);
  if (!wsConnected) startDemo();
  startRideClock();
  updateRideStatusUI();
}

function updateClock() {
  $("clock").textContent = new Intl.DateTimeFormat([], {
    hour: "2-digit", minute: "2-digit"
  }).format(new Date());
}

async function initSession() {
  try {
    const res = await fetch("/api/device/config", { credentials: "same-origin" });
    if (res.ok) {
      const data = await res.json();
      if (data.ok) {
        deviceMode = data.deviceMode === "connected" ? "connected" : "demo";
        wsUrl = data.wsUrl || "";
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
