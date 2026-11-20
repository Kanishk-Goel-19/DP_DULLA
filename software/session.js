const $ = (id) => document.getElementById(id);
const video = $("backgroundVideo");
const youtubeFrame = $("youtubeBackground");

const CYCLING_MODES = {
  torque: {
    key: "torque",
    wireMode: "cycle_torque",
    opMode: 7, // OP_CYCLE_TORQUE
    title: "Constant Torque",
    controlLabel: "Target Torque",
    unit: "Nm",
    min: 2,
    max: 85,
    step: 2,
    defaultVal: 15,
    presets: [8, 15, 25, 40]
  },
  power: {
    key: "power",
    wireMode: "cycle_power",
    opMode: 8, // OP_CYCLE_POWER
    title: "Constant Power",
    controlLabel: "Target Power",
    unit: "W",
    min: 15,
    max: 250,
    step: 10,
    defaultVal: 60,
    presets: [40, 60, 100, 150]
  },
  isokinetic: {
    key: "isokinetic",
    wireMode: "cycle_isokinetic",
    opMode: 14, // OP_CYCLE_ISOKINETIC
    title: "Isokinetic Cadence",
    controlLabel: "Target Cadence",
    unit: "RPM",
    min: 15,
    max: 100,
    step: 5,
    defaultVal: 45,
    presets: [35, 45, 60, 75]
  },
  manual: {
    key: "manual",
    wireMode: "cycle_manual",
    opMode: 15, // OP_CYCLE_MANUAL
    title: "Manual Control",
    controlLabel: "Brake Angle",
    unit: "°",
    min: 0,
    max: 42,
    step: 0.3,
    defaultVal: 22.5,
    presets: []
  }
};

let socket = null;
let deviceMode = "demo";
let wsUrl = "";
let bridgeWsPath = "";
let wsConnected = false;
let u3Linked = false;
let enc2Active = false;
let hadLiveEsp = false;
let demoTimer = null;
let rideClockTimer = null;
let motionFrame = null;

let selectedModeKey = "torque";
let targetValues = {
  torque: 15,
  power: 60,
  isokinetic: 45,
  manual: 22.5
};
let stepperAngleDeg = 0;
let livePowerW = 0;

let prescription = null;
let clinicalIntake = null;

let rideStarted = false;
let paused = false;
let ended = false;
let saving = false;
let saved = false;

let activeMs = 0;
let lastTickAt = 0;
let sessionRevs = 0;
let torqueSum = 0;
let torqueSamples = 0;

// Start at 0 (real hardware telemetry will populate live values; Demo mode animates only in Demo)
let rpm = 0, speed = 0, torque = 0;
let visual = { rpm: 0, speed: 0, torque: 0 };
let animationFrom = { ...visual };
let animationTo = { ...visual };
let animationStarted = 0;
const animationDuration = 260;

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
const easeOutCubic = (t) => 1 - Math.pow(1 - t, 3);
const currentMode = () => CYCLING_MODES[selectedModeKey] || CYCLING_MODES.torque;

function buildTicks(groupId, cx, cy, radius, count, innerLength, outerLength) {
  const group = $(groupId);
  if (!group) return;
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
  $("torqueValue").textContent = values.torque >= 100 ? Math.round(values.torque) : values.torque.toFixed(1);

  $("rpmProgress").style.strokeDashoffset = 1000 * (1 - clamp(values.rpm / 120, 0, 1));
  $("speedProgress").style.strokeDashoffset = 1000 * (1 - clamp(values.speed / 40, 0, 1));
  $("torqueProgress").style.strokeDashoffset = 1000 * (1 - clamp(values.torque / 85, 0, 1));
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
    rpm: clamp(Number(next.rpm) || 0, 0, 140),
    speed: clamp(Number(next.speed) || 0, 0, 40),
    torque: clamp(Number(next.torque) || 0, 0, 85)
  };
  animationStarted = performance.now();
  if (motionFrame === null) motionFrame = requestAnimationFrame(animateMetrics);
}

// =========================================================================
// STAGE 1: MODE SELECTOR UI
// =========================================================================
function formatTargetDisplay(modeKey) {
  const m = CYCLING_MODES[modeKey] || CYCLING_MODES.torque;
  const val = targetValues[m.key];
  if (m.key === "manual") return `${stepperAngleDeg.toFixed(1)}°`;
  return `${Math.round(val)} ${m.unit}`;
}

function renderSetupUI() {
  const mode = currentMode();
  document.querySelectorAll(".cmode-card").forEach((card) => {
    const isSel = card.dataset.cmode === selectedModeKey;
    card.classList.toggle("selected", isSel);
    card.setAttribute("aria-checked", isSel ? "true" : "false");
  });

  const targetGroup = $("setupTargetControls");
  const manualNote = $("setupManualNote");
  const labelEl = $("setupTargetLabel");
  const valEl = $("setupTargetValue");
  const presetsEl = $("setupTargetPresets");
  const minusBtn = $("setupTargetMinus");
  const plusBtn = $("setupTargetPlus");

  if (mode.key === "manual") {
    if (targetGroup) targetGroup.hidden = true;
    if (manualNote) manualNote.hidden = false;
  } else {
    if (targetGroup) targetGroup.hidden = false;
    if (manualNote) manualNote.hidden = true;
    const currentVal = clamp(targetValues[mode.key], mode.min, mode.max);
    targetValues[mode.key] = currentVal;

    if (labelEl) labelEl.textContent = mode.controlLabel;
    if (valEl) valEl.textContent = `${Math.round(currentVal)} ${mode.unit}`;
    if (minusBtn) minusBtn.disabled = currentVal <= mode.min;
    if (plusBtn) plusBtn.disabled = currentVal >= mode.max;

    if (presetsEl) {
      presetsEl.textContent = "";
      mode.presets.forEach((p) => {
        const btn = document.createElement("button");
        btn.type = "button";
        btn.className = `setup-preset-btn${Math.round(currentVal) === p ? " active" : ""}`;
        btn.textContent = `${p} ${mode.unit}`;
        btn.addEventListener("click", () => {
          targetValues[mode.key] = p;
          renderSetupUI();
        });
        presetsEl.appendChild(btn);
      });
    }
  }

  updateConnectionPills();
}

function selectCyclingMode(modeKey) {
  if (!CYCLING_MODES[modeKey]) return;
  selectedModeKey = modeKey;
  renderSetupUI();
}

function updateConnectionPills() {
  const setupPill = $("setupConnectionPill");
  if (!setupPill) return;
  setupPill.classList.remove("is-live");
  if (deviceMode === "demo") {
    setupPill.textContent = "● DEMO MODE";
  } else if (wsConnected && u3Linked) {
    setupPill.textContent = "● ESP32 + CYCLING U3 LIVE";
    setupPill.classList.add("is-live");
  } else if (wsConnected) {
    setupPill.textContent = "● MASTER ESP32 LIVE · SCANNING U3";
    setupPill.classList.add("is-live");
  } else {
    setupPill.textContent = "● CONNECTING TO ESP32…";
  }
}

// =========================================================================
// STAGE 2: LIVE HUD STATUS & CONTROLS
// =========================================================================
function updateRideStatusUI() {
  const pill = $("rideStatusPill");
  const modePill = $("activeCyclingModePill");
  const pauseBtn = $("pauseResumeBtn");
  const note = $("espConnectionNote");
  const mode = currentMode();

  if (modePill) modePill.textContent = mode.title;

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
  } else if (wsConnected && u3Linked) {
    pill.textContent = "ESP32 + U3 Live";
    pill.classList.add("is-live");
    pauseBtn.textContent = "Pause Session";
    pauseBtn.disabled = false;
  } else if (wsConnected) {
    pill.textContent = "Master Live · Wait U3";
    pill.classList.add("is-live");
    pauseBtn.textContent = "Pause Session";
    pauseBtn.disabled = false;
  } else if (deviceMode === "connected") {
    pill.textContent = "Connecting ESP32…";
    pauseBtn.textContent = "Pause Session";
    pauseBtn.disabled = false;
  } else {
    pill.textContent = "Demo Mode";
    pauseBtn.textContent = "Pause Session";
    pauseBtn.disabled = false;
  }

  $("rideTimerText").textContent = formatDuration(activeMs);
  $("rideRepsText").textContent = `${Math.floor(sessionRevs)} revs`;
  if ($("ridePowerText")) $("ridePowerText").textContent = `${Math.round(livePowerW)} W`;
  if ($("rideAngleText")) $("rideAngleText").textContent = `${stepperAngleDeg.toFixed(1)}°`;

  if (wsConnected && u3Linked) {
    note.textContent = `Live ESP32 + Cycling U3 telemetry (${wsUrl}).`;
  } else if (wsConnected) {
    note.textContent = `Master ESP32 connected (${wsUrl}). Waiting for Cycling U3 ESP-NOW link…`;
  } else if (deviceMode === "connected" && wsUrl) {
    note.textContent = `Connecting to Master ESP32 at ${wsUrl}…`;
  } else {
    note.textContent = "Demo mode active — cycling metrics are simulated locally.";
  }

  updateConnectionPills();
  updateCyclingControlUI();
}

function updateCyclingControlUI() {
  const mode = currentMode();
  const labelEl = $("cyclingControlLabel");
  const valEl = $("cyclingTargetTorqueValue");
  const minusBtn = $("cyclingTorqueMinusBtn");
  const plusBtn = $("cyclingTorquePlusBtn");
  const statusEl = $("cyclingCmdStatus");

  if (labelEl) labelEl.textContent = mode.controlLabel;
  if (valEl) valEl.textContent = formatTargetDisplay(mode.key);

  if (mode.key === "manual") {
    if (minusBtn) minusBtn.disabled = ended || stepperAngleDeg <= 0.05;
    if (plusBtn) plusBtn.disabled = ended || stepperAngleDeg >= 41.95;
  } else {
    const val = targetValues[mode.key];
    if (minusBtn) minusBtn.disabled = val <= mode.min || ended;
    if (plusBtn) plusBtn.disabled = val >= mode.max || ended;
  }

  if (statusEl && !statusEl.dataset.lockedMessage) {
    if (ended) {
      statusEl.textContent = "Session ended · Brake returned to 0.0°";
    } else if (wsConnected && u3Linked) {
      statusEl.textContent = mode.key === "manual"
        ? `U3 Live · Stepper ${stepperAngleDeg.toFixed(1)}° (±0.30°/step)`
        : `U3 Live · ${mode.title} · Brake ${stepperAngleDeg.toFixed(1)}°`;
    } else if (wsConnected) {
      statusEl.textContent = "Master linked · Waiting for Cycling U3…";
    } else if (deviceMode === "connected") {
      statusEl.textContent = "Reconnecting to Master ESP32…";
    } else {
      statusEl.textContent = `Demo mode · ${mode.title}`;
    }
  }
}

function buildCyclingStartPayload() {
  const mode = currentMode();
  const cyc = prescription?.cycling;
  return {
    type: "start_session",
    mode: mode.wireMode,
    op_mode: mode.opMode,
    role: "resistance",
    torque_nm: targetValues.torque,
    target_torque_ext: targetValues.torque,
    target_power_w: targetValues.power,
    target_cadence_rpm: targetValues.isokinetic,
    target_reps: cyc?.suggestedCycles || 25,
    pathology: clinicalIntake?.pathology ?? null,
    d_offset_m: Number(((clinicalIntake?.dOffsetCm ?? 9.0) / 100).toFixed(3)),
    L_leg_m: Number(((clinicalIntake?.lLegCm ?? 30.0) / 100).toFixed(3))
  };
}

function adjustActiveCyclingControl(directionSign) {
  if (ended) return;
  const mode = currentMode();
  const statusEl = $("cyclingCmdStatus");
  if (statusEl) delete statusEl.dataset.lockedMessage;

  if (mode.key === "manual") {
    const nextAngle = clamp(stepperAngleDeg + directionSign * mode.step, mode.min, mode.max);
    if (!wsConnected) {
      stepperAngleDeg = nextAngle;
    }
    const { sentLive } = sendEspCommand({
      type: "set_torque",
      mode: mode.wireMode,
      op_mode: mode.opMode,
      role: "resistance",
      delta: directionSign,
      step_action: directionSign > 0 ? "plus" : "minus"
    });
    if (statusEl) {
      statusEl.dataset.lockedMessage = "1";
      statusEl.textContent = sentLive
        ? `Sent ${directionSign > 0 ? "+1" : "-1"} step to Cycling U3`
        : `Manual step ${directionSign > 0 ? "+" : "-"}${mode.step.toFixed(2)}° (demo)`;
      setTimeout(() => {
        if (statusEl) delete statusEl.dataset.lockedMessage;
        updateCyclingControlUI();
      }, 900);
    }
    updateCyclingControlUI();
    return;
  }

  const next = clamp(targetValues[mode.key] + directionSign * mode.step, mode.min, mode.max);
  if (next === targetValues[mode.key]) return;
  targetValues[mode.key] = next;

  const { sentLive } = sendEspCommand({
    type: "set_torque",
    mode: mode.wireMode,
    op_mode: mode.opMode,
    role: "resistance",
    torque_nm: targetValues.torque,
    target_torque_ext: targetValues.torque,
    target_power_w: targetValues.power,
    target_cadence_rpm: targetValues.isokinetic,
    delta: directionSign * mode.step
  });

  if (statusEl) {
    statusEl.dataset.lockedMessage = "1";
    statusEl.textContent = sentLive
      ? `Sent to ESP: ${Math.round(next)} ${mode.unit}`
      : `Target set to ${Math.round(next)} ${mode.unit} (demo)`;
    setTimeout(() => {
      if (statusEl) delete statusEl.dataset.lockedMessage;
      updateCyclingControlUI();
    }, 1000);
  }
  updateCyclingControlUI();
}

// =========================================================================
// REAL ESP32 TELEMETRY & DEMO SIMULATION
// =========================================================================
function updateFromESP(data) {
  if (!data || typeof data !== "object") return;

  if (data.u3_linked !== undefined) {
    u3Linked = Number(data.u3_linked) === 1;
  }
  if (data.enc2_active !== undefined) {
    enc2Active = Number(data.enc2_active) === 1;
  }
  if (data.stepper_angle_deg !== undefined && Number.isFinite(Number(data.stepper_angle_deg))) {
    stepperAngleDeg = Number(data.stepper_angle_deg);
  }
  if (data.user_power_watts !== undefined && Number.isFinite(Number(data.user_power_watts))) {
    livePowerW = Math.max(0, Number(data.user_power_watts));
  }

  updateConnectionPills();

  if (!rideStarted || paused || ended) {
    if (rideStarted) updateRideStatusUI();
    return;
  }

  const rawRpm = data.rpm ?? data.pedal_rpm ?? data.cadence;
  const rawSpeed = data.speed ?? data.speed_kmh;
  const rawTorque = data.torque ?? data.user_pedal_torque ?? data.eddy_brake_torque ?? data.torque_nm;
  const rawReps = data.reps ?? data.completed_revs ?? data.pedal_cycles ?? data.cycles;

  const hasRpm = rawRpm !== undefined && Number.isFinite(Number(rawRpm));
  const hasSpeed = rawSpeed !== undefined && Number.isFinite(Number(rawSpeed));
  const hasTorque = rawTorque !== undefined && Number.isFinite(Number(rawTorque));
  if (!hasRpm && !hasSpeed && !hasTorque && rawReps === undefined) return;

  const next = {
    rpm: hasRpm ? Math.max(0, Number(rawRpm)) : animationTo.rpm,
    speed: hasSpeed ? Math.max(0, Number(rawSpeed)) : animationTo.speed,
    torque: hasTorque ? Math.max(0, Number(rawTorque)) : animationTo.torque
  };
  rpm = next.rpm;
  speed = next.speed;
  torque = next.torque;

  if (next.rpm > 0.5 || next.torque > 0.1) {
    torqueSum += next.torque;
    torqueSamples += 1;
  }
  if (rawReps !== undefined && Number.isFinite(Number(rawReps))) {
    sessionRevs = Math.max(0, Number(rawReps));
  }
  setMetricTargets(next);
  updateRideStatusUI();
}

function startDemo() {
  stopDemo();
  // STRICT: Never run dummy simulation when connected to real ESP32 hardware
  if (deviceMode !== "demo" || !rideStarted || paused || ended || wsConnected) return;

  const mode = currentMode();
  if (rpm === 0) {
    rpm = mode.key === "isokinetic" ? targetValues.isokinetic : 58;
    speed = rpm * 0.31;
    torque = mode.key === "torque" ? targetValues.torque : 16;
    stepperAngleDeg = mode.key === "manual" ? 22.5 : 18.0;
    livePowerW = torque * (rpm * 2 * Math.PI / 60);
    setMetricTargets({ rpm, speed, torque });
  }

  demoTimer = setInterval(() => {
    if (deviceMode !== "demo" || !rideStarted || paused || ended || wsConnected) return;
    const m = currentMode();
    if (m.key === "isokinetic") {
      rpm = clamp(targetValues.isokinetic + (Math.random() - 0.5) * 3, 15, 100);
      torque = clamp(18 + (rpm - targetValues.isokinetic) * 2.5 + (Math.random() - 0.5) * 2, 4, 70);
    } else if (m.key === "power") {
      rpm = clamp(rpm + (Math.random() - 0.5) * 4, 42, 82);
      const omega = Math.max(2, rpm * (2 * Math.PI / 60));
      torque = clamp(targetValues.power / omega + (Math.random() - 0.5) * 1.2, 2, 85);
    } else if (m.key === "manual") {
      rpm = clamp(rpm + (Math.random() - 0.5) * 3.5, 45, 78);
      torque = clamp((stepperAngleDeg / 42) * 38 + (Math.random() - 0.5) * 1.5, 0, 85);
    } else {
      rpm = clamp(rpm + (Math.random() - 0.5) * 4, 42, 85);
      torque = clamp(targetValues.torque + (Math.random() - 0.5) * 1.8, 2, 85);
    }
    speed = clamp(rpm * 0.31, 0, 40);
    livePowerW = torque * (rpm * 2 * Math.PI / 60);
    if (m.key !== "manual") {
      stepperAngleDeg = clamp((torque / 45) * 32, 5, 42);
    }
    torqueSum += torque;
    torqueSamples += 1;
    setMetricTargets({ rpm, speed, torque });
    updateRideStatusUI();
  }, 300);
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
    if (rideStarted && !paused && !ended) {
      activeMs += dt;
      if (deviceMode === "demo" && !wsConnected) {
        sessionRevs += (rpm / 60) * (dt / 1000);
      }
      $("rideTimerText").textContent = formatDuration(activeMs);
      $("rideRepsText").textContent = `${Math.floor(sessionRevs)} revs`;
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

// =========================================================================
// STAGE TRANSITIONS (SETUP <-> LIVE RIDE)
// =========================================================================
function enterCyclingRide() {
  rideStarted = true;
  paused = false;
  ended = false;
  saved = false;
  saving = false;
  activeMs = 0;
  sessionRevs = 0;
  torqueSum = 0;
  torqueSamples = 0;
  rpm = 0;
  speed = 0;
  torque = 0;
  livePowerW = 0;
  visual = { rpm: 0, speed: 0, torque: 0 };
  animationFrom = { ...visual };
  animationTo = { ...visual };
  paintMetrics(visual);

  if (selectedModeKey === "manual" && !wsConnected) {
    stepperAngleDeg = 22.5;
  }

  $("cyclingSetupView").hidden = true;
  $("dashboard").hidden = false;
  $("sessionSummaryModal").hidden = true;
  document.body.classList.add("is-riding");

  const statusEl = $("cyclingCmdStatus");
  if (statusEl) delete statusEl.dataset.lockedMessage;

  changeScene(document.querySelector(".scene-option.active")?.dataset.scene || "forest");
  controlBackgroundPlayback(true);
  startRideClock();

  if (wsConnected) {
    sendEspCommand(buildCyclingStartPayload());
  } else if (deviceMode === "demo") {
    startDemo();
  }
  updateRideStatusUI();
}

function returnToModeSelector() {
  if (rideStarted && !ended) {
    sendEspCommand({ type: "stop_session" });
  }
  rideStarted = false;
  paused = false;
  ended = false;
  stopDemo();
  stopRideClock();
  controlBackgroundPlayback(false);
  $("menuPanel").classList.remove("open");
  $("sessionSummaryModal").hidden = true;
  $("dashboard").hidden = true;
  $("cyclingSetupView").hidden = false;
  document.body.classList.remove("is-riding");
  renderSetupUI();
  window.scrollTo({ top: 0, behavior: "instant" });
}

// =========================================================================
// SCENERY & WEBSOCKET CONNECTION
// =========================================================================
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
        if (rideStarted && !paused && !ended) {
          sendEspCommand(buildCyclingStartPayload());
        }
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
          if (!wsConnected) {
            u3Linked = false;
            enc2Active = false;
          }
          const statusEl = $("cyclingCmdStatus");
          if (statusEl) delete statusEl.dataset.lockedMessage;
          if (wsConnected) {
            stopDemo();
            if (rideStarted && !paused && !ended) sendEspCommand(buildCyclingStartPayload());
            if (rideStarted && paused) sendEspCommand({ type: "pause_session" });
          }
          updateRideStatusUI();
          return;
        }

        if (packet.type === "esp_ack") {
          if (packet.forwarded === false && $("cyclingCmdStatus")) {
            $("cyclingCmdStatus").dataset.lockedMessage = "1";
            $("cyclingCmdStatus").textContent = packet.message || "ESP32 offline — waiting for connection";
          }
          return;
        }

        hadLiveEsp = true;
        if (!wsConnected) {
          wsConnected = true;
          stopDemo();
          const statusEl = $("cyclingCmdStatus");
          if (statusEl) delete statusEl.dataset.lockedMessage;
        }
        updateFromESP(packet);
      } catch (error) {
        console.warn("Invalid ESP32 data packet", error);
      }
    });
    socket.addEventListener("close", () => {
      wsConnected = false;
      u3Linked = false;
      enc2Active = false;
      updateRideStatusUI();
    });
    socket.addEventListener("error", () => {
      wsConnected = false;
      u3Linked = false;
      updateRideStatusUI();
    });
  } catch {
    wsConnected = false;
  }
}

// =========================================================================
// SESSION PERSISTENCE, PAUSE & END
// =========================================================================
async function persistCyclingSession() {
  if (saving || saved) return;
  saving = true;
  const mode = currentMode();
  const statusEl = $("summarySaveStatus");
  const retryBtn = $("retrySaveBtn");
  statusEl.className = "summary-status";
  statusEl.textContent = "Saving session to Track Progress…";
  retryBtn.hidden = true;

  const durationSeconds = Math.max(0, Math.round(activeMs / 1000));
  const reps = clamp(Math.round(sessionRevs), 1, 999);
  const avgTorque = Math.round((torqueSamples ? torqueSum / torqueSamples : torque) * 10) / 10;

  if ($("summaryModeEyebrow")) {
    $("summaryModeEyebrow").textContent = `DOULA · Cycling (${mode.title})`;
  }
  $("summaryDuration").textContent = formatDuration(durationSeconds * 1000);
  $("summaryReps").textContent = String(Math.floor(sessionRevs));
  $("summaryTorque").textContent = `${avgTorque.toFixed(1)} Nm`;

  try {
    const response = await fetch("/api/progress", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      credentials: "same-origin",
      body: JSON.stringify({
        type: "Cycling",
        mode: `Cycling · ${mode.title}`,
        torqueNm: avgTorque,
        reps: clamp(reps, 1, 99),
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
  if (!rideStarted || ended) return;
  paused = !paused;
  lastTickAt = performance.now();
  if (paused) {
    stopDemo();
    controlBackgroundPlayback(false);
    sendEspCommand({ type: "pause_session" });
  } else {
    controlBackgroundPlayback(true);
    sendEspCommand({ type: "resume_session" });
    if (deviceMode === "demo" && !wsConnected) startDemo();
  }
  updateRideStatusUI();
}

function endCyclingSession() {
  if (!rideStarted || ended) return;
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

function updateClock() {
  $("clock").textContent = new Intl.DateTimeFormat([], {
    hour: "2-digit", minute: "2-digit"
  }).format(new Date());
}

// =========================================================================
// CLINICAL PRESCRIPTION PRE-FILL
// =========================================================================
function inferRecommendedCyclingMode(cyc) {
  if (!cyc || !cyc.modeName) return "torque";
  const m = String(cyc.modeName).toLowerCase();
  if (m.includes("isok")) return "isokinetic";
  if (m.includes("power") || m.includes("ergometer")) return "power";
  return "torque";
}

function renderCyclingPrescription(rx) {
  const banner = $("cyclingRecBanner");
  const menuNote = $("cyclingRxNote");
  const setupStrip = $("setupRxStrip");
  const setupBadge = $("setupRxBadge");
  const setupCopy = $("setupRxCopy");
  const setupLink = $("setupRxLink");

  document.querySelectorAll("[data-cmode-rec]").forEach((el) => { el.hidden = true; });

  if (!rx || !rx.cycling) {
    if (banner) banner.hidden = true;
    if (menuNote) menuNote.hidden = true;
    if (setupStrip) setupStrip.hidden = true;
    return;
  }

  const cyc = rx.cycling;
  const recKey = inferRecommendedCyclingMode(cyc);

  if (cyc.allowed) {
    const recBadge = document.querySelector(`[data-cmode-rec="${recKey}"]`);
    if (recBadge) recBadge.hidden = false;
    selectedModeKey = recKey;

    const rxTorque = Number(cyc.targetTorqueNm);
    if (Number.isFinite(rxTorque) && rxTorque >= CYCLING_MODES.torque.min) {
      targetValues.torque = clamp(Math.round(rxTorque), CYCLING_MODES.torque.min, CYCLING_MODES.torque.max);
    }
    const powerMatch = /(\d+)\s*[–-]\s*(\d+)\s*W/i.exec(String(cyc.powerRangeW || ""));
    if (powerMatch) {
      const midP = Math.round((Number(powerMatch[1]) + Number(powerMatch[2])) / 2);
      targetValues.power = clamp(midP, CYCLING_MODES.power.min, CYCLING_MODES.power.max);
    }
    const cadMatch = /(\d+)\s*[–-]\s*(\d+)/.exec(String(cyc.cadenceRangeRpm || ""));
    if (cadMatch) {
      const midC = Math.round((Number(cadMatch[1]) + Number(cadMatch[2])) / 2);
      targetValues.isokinetic = clamp(midC, CYCLING_MODES.isokinetic.min, CYCLING_MODES.isokinetic.max);
    }
  }

  if (setupStrip && setupBadge && setupCopy && setupLink) {
    setupStrip.hidden = false;
    setupStrip.classList.toggle("is-caution", !cyc.allowed);
    setupBadge.textContent = cyc.allowed ? "★ Suggested" : "Clinical Note";
    setupCopy.textContent = cyc.allowed
      ? `${cyc.modeName} · ${cyc.targetTorqueNm} Nm · ${cyc.cadenceRangeRpm}`
      : `${cyc.priorityLabel} — ${cyc.note}`;
    setupLink.href = cyc.allowed ? "/intake" : "/device/flexion/";
    setupLink.textContent = cyc.allowed ? "Edit Intake" : "Go to Flexion →";
  }

  if (banner) {
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
      ? `${cyc.modeName} · ${cyc.targetTorqueNm} Nm · ${cyc.cadenceRangeRpm}`
      : `${cyc.priorityLabel} — ${cyc.note}`;

    textWrap.append(badge, msg);

    const action = document.createElement("a");
    action.className = "cycling-rx-action";
    action.href = cyc.allowed ? "/intake" : "/device/flexion/";
    action.textContent = cyc.allowed ? "Update Intake →" : "Switch to Flexion →";

    banner.append(textWrap, action);
  }

  if (menuNote) {
    menuNote.hidden = false;
    menuNote.textContent = cyc.allowed
      ? `Rx: ${cyc.modeName}, ${cyc.targetTorqueNm} Nm, ${cyc.cadenceRangeRpm}.`
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
        renderCyclingPrescription(prescription);
      }
    }
  } catch {}

  renderSetupUI();
  connectEspSocket();
}

// =========================================================================
// EVENT LISTENERS
// =========================================================================
document.querySelectorAll(".cmode-card").forEach((card) => {
  card.addEventListener("click", () => selectCyclingMode(card.dataset.cmode));
});

$("setupTargetMinus")?.addEventListener("click", () => {
  const mode = currentMode();
  if (mode.key === "manual") return;
  targetValues[mode.key] = clamp(targetValues[mode.key] - mode.step, mode.min, mode.max);
  renderSetupUI();
});

$("setupTargetPlus")?.addEventListener("click", () => {
  const mode = currentMode();
  if (mode.key === "manual") return;
  targetValues[mode.key] = clamp(targetValues[mode.key] + mode.step, mode.min, mode.max);
  renderSetupUI();
});

$("startCyclingBtn")?.addEventListener("click", enterCyclingRide);
$("changeModeBtn")?.addEventListener("click", returnToModeSelector);
$("menuChangeModeBtn")?.addEventListener("click", returnToModeSelector);
$("summaryChangeModeBtn")?.addEventListener("click", returnToModeSelector);

$("menuButton").addEventListener("click", () => $("menuPanel").classList.toggle("open"));
$("pauseResumeBtn").addEventListener("click", togglePauseSession);
$("endSessionBtn").addEventListener("click", endCyclingSession);
$("retrySaveBtn").addEventListener("click", () => void persistCyclingSession());
$("restartRideBtn").addEventListener("click", enterCyclingRide);
$("cyclingTorqueMinusBtn")?.addEventListener("click", () => adjustActiveCyclingControl(-1));
$("cyclingTorquePlusBtn")?.addEventListener("click", () => adjustActiveCyclingControl(1));

$("fullscreenButton").addEventListener("click", async () => {
  try {
    if (!document.fullscreenElement) await $("dashboard").requestFullscreen();
    else await document.exitFullscreen();
  } catch (error) {
    console.warn("Fullscreen unavailable", error);
  }
});

$("switchSessionButton").addEventListener("click", () => {
  if (rideStarted && !ended) sendEspCommand({ type: "stop_session" });
  window.location.assign("/device/");
});
$("backDashboardButton").addEventListener("click", () => {
  if (rideStarted && !ended) sendEspCommand({ type: "stop_session" });
  window.location.assign("/dashboard");
});

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

updateClock();
setInterval(updateClock, 15000);
paintMetrics(visual);
renderSetupUI();
void initSession();

window.doulaCycling = { updateFromESP };
