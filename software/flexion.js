(function () {
  "use strict";

  const MODE_CATALOG = {
    passive: {
      key: "passive",
      opMode: 3, // OP_CPM
      title: "Passive Assist",
      clinicalLabel: "Motor-Assisted CPM Motion",
      role: "support",
      rolePillText: "Support Torque",
      odometerTitle: "Support Torque · ESP",
      odometerSubLabel: "Support Torque",
      setpointLabel: "Support Target → ESP",
      suggestedReps: 15,
      repPresets: [10, 15, 20, 25],
      minTorque: 4,
      maxTorque: 25,
      defaultTarget: 12,
      defaultVel: 18,
      step: 1,
      simAmplitude: 1.8
    },
    isotonic: {
      key: "isotonic",
      opMode: 4, // OP_ISOTONIC
      title: "Isotonic Mode",
      clinicalLabel: "Physiological Curve-Shaped Isotonic",
      role: "resistance",
      rolePillText: "Resistance Torque",
      odometerTitle: "Resistance Torque · ESP",
      odometerSubLabel: "Resistance Torque",
      setpointLabel: "Resistance Target → ESP",
      suggestedReps: 12,
      repPresets: [8, 10, 12, 15],
      minTorque: 5,
      maxTorque: 45,
      defaultTarget: 22,
      defaultVel: 60,
      step: 2,
      simAmplitude: 3.4
    },
    isokinetic: {
      key: "isokinetic",
      opMode: 5, // OP_ISOKINETIC
      title: "Isokinetic Mode",
      clinicalLabel: "Constant-Velocity Accommodating",
      role: "resistance",
      rolePillText: "Resistance Torque",
      odometerTitle: "Resistance Torque · ESP",
      odometerSubLabel: "Resistance Torque",
      setpointLabel: "Resistance Target → ESP",
      suggestedReps: 10,
      repPresets: [6, 8, 10, 12],
      minTorque: 10,
      maxTorque: 45,
      defaultTarget: 30,
      defaultVel: 45,
      step: 2,
      simAmplitude: 5.0
    },
    active_assist: {
      key: "active_assist",
      opMode: 11, // OP_ACTIVE_ASSIST
      title: "Active Assist",
      clinicalLabel: "Trigger-and-Go Active Assist",
      role: "support",
      rolePillText: "Support Torque",
      odometerTitle: "Support Torque · ESP",
      odometerSubLabel: "Support Torque",
      setpointLabel: "Support Target → ESP",
      suggestedReps: 12,
      repPresets: [8, 10, 12, 15],
      minTorque: 4,
      maxTorque: 25,
      defaultTarget: 12,
      defaultVel: 25,
      step: 1,
      simAmplitude: 2.0
    },
    isometric: {
      key: "isometric",
      opMode: 6, // OP_ISOMETRIC
      title: "Isometric Hold",
      clinicalLabel: "Isometric Angle-Lock Hold",
      role: "resistance",
      rolePillText: "Resistance Torque",
      odometerTitle: "Hold Torque · ESP",
      odometerSubLabel: "Isometric Torque",
      setpointLabel: "Hold Torque Cap → ESP",
      suggestedReps: 8,
      repPresets: [5, 8, 10, 12],
      minTorque: 5,
      maxTorque: 45,
      defaultTarget: 20,
      defaultVel: 150,
      step: 2,
      simAmplitude: 2.2
    }
  };

  const state = {
    modeKey: "isotonic",
    targetReps: MODE_CATALOG.isotonic.suggestedReps,
    completedReps: 0,
    targetTorque: MODE_CATALOG.isotonic.defaultTarget,
    liveTorque: 0,
    peakTorque: 0,
    torqueSum: 0,
    torqueSamples: 0,
    startedAtMs: 0,
    active: false,
    paused: false,
    ended: false,
    estopped: false,
    autoSim: true,
    deviceMode: "demo",
    shouldConnectWs: false,
    espConnected: false,
    u2Linked: false,
    u2Homed: false,
    u2Calibrated: false,
    u2BusyHoming: false,
    u2Positioning: false,
    u2HardStop: false,
    u2Spasm: false,
    u2FaultLockout: false,
    setupState: "idle",
    setupWarning: false,
    setupTargetDeg: 120,
    unhomedPrompt: false,
    lastSafetyMessage: "",
    kneeAngleDeg: 0,
    kneeVelDegS: 0,
    vbusVolts: 0,
    motorCurrentA: 0,
    lastRepWorkJ: 0,
    wsUrl: null,
    clinicalRx: null,
    romWindowDeg: [120, 180],
    targetVelDegS: 60,
    dOffsetM: 0.250,
    lLegM: 0.300,
    pathologyCode: 0,
    simPhase: 0,
    simRepElapsedMs: 0,
    sessionSaved: false,
    savingSession: false,
    lastSavePayload: null,
    lastLocalTorqueSetMs: 0,
    lastLocalKinSetMs: 0
  };

  let socket = null;
  let reconnectTimer = null;
  let simTimer = null;
  let pendingOpenCommands = [];

  // DOM References
  const menuToggleBtn = document.getElementById("menuToggleBtn");
  const mobileNavDrawer = document.getElementById("mobileNavDrawer");
  const connectionBadge = document.getElementById("connectionBadge");

  // Top-Hub Hardware Bar References
  const topHubLinkTitle = document.getElementById("topHubLinkTitle");
  const quickHomeBtn = document.getElementById("quickHomeBtn");
  const autoHomeBtn = document.getElementById("autoHomeBtn");
  const parkHomeBtn = document.getElementById("parkHomeBtn");
  const releaseLockBtn = document.getElementById("releaseLockBtn");
  const thLinkChip = document.getElementById("thLinkChip");
  const thHomeChip = document.getElementById("thHomeChip");
  const thAngleChip = document.getElementById("thAngleChip");
  const thPowerChip = document.getElementById("thPowerChip");
  const thWorkChip = document.getElementById("thWorkChip");
  const thAngleBarFill = document.getElementById("thAngleBarFill");
  const topHubStatusNote = document.getElementById("topHubStatusNote");

  // Safety / Setup Transition Banner References
  const setupWarningBanner = document.getElementById("setupWarningBanner");
  const setupBannerBadge = document.getElementById("setupBannerBadge");
  const setupBannerTitle = document.getElementById("setupBannerTitle");
  const setupBannerMessage = document.getElementById("setupBannerMessage");
  const setupBannerProgressWrap = document.getElementById("setupBannerProgressWrap");
  const setupBannerProgressFill = document.getElementById("setupBannerProgressFill");
  const setupBannerProgressText = document.getElementById("setupBannerProgressText");
  const beginExerciseBtn = document.getElementById("beginExerciseBtn");
  const cancelSetupBtn = document.getElementById("cancelSetupBtn");
  const bannerQuickHomeBtn = document.getElementById("bannerQuickHomeBtn");
  const bannerAutoHomeBtn = document.getElementById("bannerAutoHomeBtn");
  const bannerReleaseBtn = document.getElementById("bannerReleaseBtn");
  const bannerHardStopBtn = document.getElementById("bannerHardStopBtn");

  const setupView = document.getElementById("setupView");
  const activeView = document.getElementById("activeView");
  const clinicalFlexionCard = document.getElementById("clinicalFlexionCard");
  const modeCards = Array.from(document.querySelectorAll(".mode-card"));
  const extraModePills = Array.from(document.querySelectorAll(".mode-pill-extra"));
  const suggestionSourceLabel = document.getElementById("suggestionSourceLabel");
  const suggestedModeName = document.getElementById("suggestedModeName");
  const suggestedRepCount = document.getElementById("suggestedRepCount");
  const useSuggestedBtn = document.getElementById("useSuggestedBtn");
  const repPresets = document.getElementById("repPresets");
  const repMinusBtn = document.getElementById("repMinusBtn");
  const repPlusBtn = document.getElementById("repPlusBtn");
  const repTargetDisplay = document.getElementById("repTargetDisplay");
  const romMinInput = document.getElementById("romMinInput");
  const romMaxInput = document.getElementById("romMaxInput");
  const activeRomMinInput = document.getElementById("activeRomMinInput");
  const activeRomMaxInput = document.getElementById("activeRomMaxInput");
  const applyActiveRomBtn = document.getElementById("applyActiveRomBtn");
  const romPresetBtns = Array.from(document.querySelectorAll(".rom-preset-btn"));
  const velInputLabel = document.getElementById("velInputLabel");
  const velUnitText = document.getElementById("velUnitText");
  const targetVelInput = document.getElementById("targetVelInput");
  const applyKinematicsBtn = document.getElementById("applyKinematicsBtn");
  const setupSummary = document.getElementById("setupSummary");
  const startWorkoutBtn = document.getElementById("startWorkoutBtn");

  const activeModePill = document.getElementById("activeModePill");
  const espStatusPill = document.getElementById("espStatusPill");
  const backToSetupBtn = document.getElementById("backToSetupBtn");

  const repsRemainingValue = document.getElementById("repsRemainingValue");
  const repsRemainingCaption = document.getElementById("repsRemainingCaption");
  const repsProgressText = document.getElementById("repsProgressText");
  const activeSuggestedTag = document.getElementById("activeSuggestedTag");
  const repProgressFill = document.getElementById("repProgressFill");
  const sessionCompleteBanner = document.getElementById("sessionCompleteBanner");
  const bannerTitle = document.getElementById("bannerTitle");
  const bannerMessage = document.getElementById("bannerMessage");
  const repeatSessionBtn = document.getElementById("repeatSessionBtn");
  const completeParkBtn = document.getElementById("completeParkBtn");
  const chooseAnotherModeBtn = document.getElementById("chooseAnotherModeBtn");
  const repTestControls = document.getElementById("repTestControls");
  const simRepBtn = document.getElementById("simRepBtn");
  const autoSimToggleBtn = document.getElementById("autoSimToggleBtn");

  const torqueRoleEyebrow = document.getElementById("torqueRoleEyebrow");
  const torqueSourceTag = document.getElementById("torqueSourceTag");
  const odoTicks = document.getElementById("odoTicks");
  const odoProgress = document.getElementById("odoProgress");
  const odoTargetMarker = document.getElementById("odoTargetMarker");
  const torqueValue = document.getElementById("torqueValue");
  const torqueOdometerLabel = document.getElementById("torqueOdometerLabel");
  const odoMaxLabel = document.getElementById("odoMaxLabel");
  const torqueMinusBtn = document.getElementById("torqueMinusBtn");
  const torquePlusBtn = document.getElementById("torquePlusBtn");
  const setpointLabel = document.getElementById("setpointLabel");
  const targetTorqueValue = document.getElementById("targetTorqueValue");
  const espCommandStatus = document.getElementById("espCommandStatus");
  const activeTorqueRec = document.getElementById("activeTorqueRec");

  const saveFeedback = document.getElementById("saveFeedback");
  const saveFeedbackText = document.getElementById("saveFeedbackText");
  const retrySaveBtn = document.getElementById("retrySaveBtn");

  const pauseSessionBtn = document.getElementById("pauseSessionBtn");
  const endSessionBtn = document.getElementById("endSessionBtn");
  const hardStopBtn = document.getElementById("hardStopBtn");
  const historyList = document.getElementById("historyList");

  function clamp(num, min, max) {
    return Math.min(Math.max(num, min), max);
  }

  function escapeHtml(value) {
    return String(value ?? "").replace(/[&<>"']/g, function (c) {
      return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c];
    });
  }

  function currentMode() {
    return MODE_CATALOG[state.modeKey] || MODE_CATALOG.isotonic;
  }

  function buildOdometerTicks() {
    if (!odoTicks) return;
    const cx = 130;
    const cy = 125;
    const rInner = 75;
    const rOuterMajor = 83;
    const rOuterMinor = 79;
    const startDeg = 135;
    const sweepDeg = 270;
    const count = 18;
    const lines = [];

    for (let i = 0; i <= count; i += 1) {
      const isMajor = i % 3 === 0;
      const deg = startDeg + (sweepDeg * i) / count;
      const rad = (deg * Math.PI) / 180;
      const rOut = isMajor ? rOuterMajor : rOuterMinor;
      const x1 = (cx + rInner * Math.cos(rad)).toFixed(2);
      const y1 = (cy + rInner * Math.sin(rad)).toFixed(2);
      const x2 = (cx + rOut * Math.cos(rad)).toFixed(2);
      const y2 = (cy + rOut * Math.sin(rad)).toFixed(2);
      lines.push(
        `<line class="${isMajor ? "major" : ""}" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" />`
      );
    }
    odoTicks.innerHTML = lines.join("");
  }

  function setTargetMarkerAngle(ratio) {
    if (!odoTargetMarker) return;
    const clamped = clamp(ratio, 0, 1);
    const deg = 135 + clamped * 270;
    const rad = (deg * Math.PI) / 180;
    const cx = 130;
    const cy = 125;
    const r = 92;
    const x = cx + r * Math.cos(rad);
    const y = cy + r * Math.sin(rad);
    odoTargetMarker.setAttribute("cx", x.toFixed(2));
    odoTargetMarker.setAttribute("cy", y.toFixed(2));
  }

  function setOdometerValue(nm) {
    const mode = currentMode();
    const safeNm = clamp(Number(nm) || 0, 0, mode.maxTorque);
    state.liveTorque = safeNm;

    if (state.active && !state.paused && !state.ended && safeNm > 0) {
      if (safeNm > state.peakTorque) {
        state.peakTorque = safeNm;
      }
      state.torqueSum += safeNm;
      state.torqueSamples += 1;
    }

    if (torqueValue) {
      torqueValue.textContent = safeNm.toFixed(1);
    }
    if (odoProgress) {
      const ratio = clamp(safeNm / mode.maxTorque, 0, 1);
      const offset = 1000 - ratio * 1000;
      odoProgress.style.strokeDashoffset = offset.toFixed(1);
    }
  }

  function updateTargetTorqueUI() {
    const mode = currentMode();
    state.targetTorque = clamp(state.targetTorque, mode.minTorque, mode.maxTorque);
    if (targetTorqueValue) {
      targetTorqueValue.textContent = `${state.targetTorque} N·m`;
    }
    if (odoMaxLabel) {
      odoMaxLabel.textContent = String(mode.maxTorque);
    }
    setTargetMarkerAngle(state.targetTorque / mode.maxTorque);

    if (torqueMinusBtn) {
      torqueMinusBtn.disabled = state.targetTorque <= mode.minTorque || state.estopped;
    }
    if (torquePlusBtn) {
      torquePlusBtn.disabled = state.targetTorque >= mode.maxTorque || state.estopped;
    }
  }

  function renderRepPresets() {
    if (!repPresets) return;
    const mode = currentMode();
    repPresets.innerHTML = "";
    mode.repPresets.forEach((count) => {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = `rep-preset-btn${count === state.targetReps ? " active" : ""}`;
      btn.textContent = `${count} reps`;
      btn.addEventListener("click", () => {
        setTargetReps(count);
      });
      repPresets.appendChild(btn);
    });
  }

  function normalizeWorkingRom(rawMin, rawMax) {
    const rMinRaw = Number.isFinite(Number(rawMin)) ? Math.round(Number(rawMin)) : 120;
    const rMin = clamp(rMinRaw, 120, 170);
    const rMaxRaw = Number.isFinite(Number(rawMax)) ? Math.round(Number(rawMax)) : 180;
    const rMax = clamp(rMaxRaw, rMin + 10, 180);
    return [rMin, rMax];
  }

  function mapAnatomicalToMachineRom(romExt, romFlex) {
    const extOffset = clamp(Math.round(Number(romExt) || 0), 0, 40);
    const flexSpan = clamp(Math.round(Number(romFlex) || 60), 10, 60);
    const minDeg = clamp(120 + extOffset, 120, 170);
    const maxDeg = clamp(120 + flexSpan, minDeg + 10, 180);
    return [minDeg, maxDeg];
  }

  function syncKinematicsInputs() {
    const mode = currentMode();
    const [rMin, rMax] = normalizeWorkingRom(state.romWindowDeg[0], state.romWindowDeg[1]);
    state.romWindowDeg = [rMin, rMax];
    if (romMinInput) romMinInput.value = String(rMin);
    if (romMaxInput) romMaxInput.value = String(rMax);
    if (activeRomMinInput) activeRomMinInput.value = String(rMin);
    if (activeRomMaxInput) activeRomMaxInput.value = String(rMax);
    if (activeSuggestedTag) {
      const sourcePrefix = state.clinicalRx?.flexion ? "Clinical Rx" : "Therapist suggested";
      activeSuggestedTag.textContent = `${sourcePrefix}: ${mode.suggestedReps} reps · ${rMin}°–${rMax}°`;
    }

    romPresetBtns.forEach((btn) => {
      const pMin = Number(btn.getAttribute("data-rom-min"));
      const pMax = Number(btn.getAttribute("data-rom-max"));
      btn.classList.toggle("active", pMin === rMin && pMax === rMax);
    });

    if (velInputLabel && velUnitText) {
      if (mode.key === "isometric") {
        velInputLabel.textContent = "Isometric Lock Angle";
        velUnitText.textContent = "Hold Angle (°)";
        state.targetVelDegS = clamp(Math.round(Number(state.targetVelDegS) || 150), rMin, rMax);
        if (targetVelInput) {
          targetVelInput.min = String(rMin);
          targetVelInput.max = String(rMax);
          targetVelInput.step = "1";
        }
      } else {
        velInputLabel.textContent = "Target Angular Velocity";
        velUnitText.textContent = "Speed (°/s)";
        state.targetVelDegS = clamp(Math.round(Number(state.targetVelDegS) || mode.defaultVel), 5, 180);
        if (targetVelInput) {
          targetVelInput.min = "5";
          targetVelInput.max = "180";
          targetVelInput.step = "5";
        }
      }
    }
    if (targetVelInput) {
      targetVelInput.value = String(Math.round(Number(state.targetVelDegS) || mode.defaultVel));
    }
  }

  function setTargetReps(nextReps) {
    state.targetReps = clamp(Math.round(Number(nextReps) || 1), 1, 50);
    if (repTargetDisplay) {
      repTargetDisplay.textContent = String(state.targetReps);
    }
    renderRepPresets();
    updateSetupSummary();
  }

  function updateSetupSummary() {
    const mode = currentMode();
    if (suggestedModeName) {
      suggestedModeName.textContent = mode.title;
    }
    if (suggestedRepCount) {
      suggestedRepCount.textContent = `${mode.suggestedReps} reps`;
    }
    if (setupSummary) {
      setupSummary.innerHTML =
        `<strong>${mode.title}</strong> · ${state.targetReps} reps · ${state.targetTorque} N·m`;
    }
  }

  function selectMode(modeKey, preserveClinicalTarget) {
    if (!MODE_CATALOG[modeKey]) return;
    state.modeKey = modeKey;
    const mode = currentMode();

    if (!preserveClinicalTarget) {
      state.targetReps = mode.suggestedReps;
      state.targetTorque = mode.defaultTarget;
      state.targetVelDegS = mode.defaultVel;
    } else {
      state.targetTorque = clamp(state.targetTorque, mode.minTorque, mode.maxTorque);
    }

    modeCards.forEach((card) => {
      const isSelected = card.getAttribute("data-mode") === modeKey;
      card.classList.toggle("selected", isSelected);
      card.setAttribute("aria-checked", isSelected ? "true" : "false");
    });

    extraModePills.forEach((pill) => {
      const isSelected = pill.getAttribute("data-mode") === modeKey;
      pill.classList.toggle("active", isSelected);
    });

    if (repTargetDisplay) {
      repTargetDisplay.textContent = String(state.targetReps);
    }
    renderRepPresets();
    syncKinematicsInputs();
    updateSetupSummary();
    updateTargetTorqueUI();

    if (state.espConnected) {
      sendEspCommand(buildSetParamsPayload("select_mode"));
    }
  }

  function renderSafetyBanner() {
    if (!setupWarningBanner) return;

    if (setupBannerProgressWrap) setupBannerProgressWrap.hidden = true;
    if (beginExerciseBtn) beginExerciseBtn.hidden = true;
    if (cancelSetupBtn) cancelSetupBtn.hidden = true;
    if (bannerQuickHomeBtn) bannerQuickHomeBtn.hidden = true;
    if (bannerAutoHomeBtn) bannerAutoHomeBtn.hidden = true;
    if (bannerReleaseBtn) bannerReleaseBtn.hidden = true;
    if (bannerHardStopBtn) bannerHardStopBtn.hidden = true;
    setupWarningBanner.className = "setup-safety-banner";

    if (state.estopped || state.u2HardStop) {
      setupWarningBanner.hidden = false;
      setupWarningBanner.classList.add("is-spasm");
      if (setupBannerBadge) setupBannerBadge.textContent = "HARD STOP";
      if (setupBannerTitle) setupBannerTitle.textContent = "HARD STOP ENGAGED — MOTOR DISABLED";
      if (setupBannerMessage) {
        setupBannerMessage.textContent = state.lastSafetyMessage ||
          "Emergency Hard-Stop is active and motor drive is disabled. Verify limb/mechanical clearance before clicking Release Lock.";
      }
      if (bannerReleaseBtn) bannerReleaseBtn.hidden = false;
      return;
    }

    if (state.u2Spasm || state.u2FaultLockout) {
      setupWarningBanner.hidden = false;
      setupWarningBanner.classList.add("is-spasm");
      if (setupBannerBadge) setupBannerBadge.textContent = state.u2Spasm ? "SPASM GUARD" : "SAFETY FAULT";
      if (setupBannerTitle) {
        setupBannerTitle.textContent = state.u2Spasm
          ? "SPASM / OBSTRUCTION DETECTED — MOTOR RELEASED"
          : "TOP-HUB SAFETY FAULT — MOTOR RELEASED";
      }
      if (setupBannerMessage) {
        setupBannerMessage.textContent = state.lastSafetyMessage ||
          "Top-Hub safely stopped the drive into zero-torque IDLE. Check patient comfort and clearance, then click Release Lock to clear the alert.";
      }
      if (bannerReleaseBtn) bannerReleaseBtn.hidden = false;
      return;
    }

    const isMovingSetup = state.setupWarning ||
      state.setupState === "moving_to_start" ||
      state.setupState === "parking_home" ||
      state.setupState === "autohoming" ||
      state.u2BusyHoming ||
      state.u2Positioning;

    if (isMovingSetup) {
      const isPark = state.setupState === "parking_home";
      const isAutoHome = state.setupState === "autohoming" || state.u2BusyHoming;
      setupWarningBanner.hidden = false;
      setupWarningBanner.classList.add("is-warning");
      if (setupBannerBadge) setupBannerBadge.textContent = "KEEP CLEAR · 10°/s";
      if (setupBannerTitle) {
        setupBannerTitle.textContent = isPark
          ? "⚠️ RETURNING TO 0° PARK — PLEASE STAY AWAY"
          : (isAutoHome
            ? "⚠️ AUTO-HOMING TO 0° STOP — PLEASE STAY AWAY"
            : "⚠️ SETUP IN PROGRESS — PLEASE STAY AWAY");
      }
      if (setupBannerMessage) {
        setupBannerMessage.textContent = isPark
          ? "Arm is slowly returning to the 0° parked stop at 10°/s. Keep hands, feet, and straps clear until it stops and releases."
          : (isAutoHome
            ? "Arm is slowly seeking the 0° STOP1 mechanical home reference. Keep limbs and obstacles clear."
            : `Setup in progress — arm is moving slowly at 10°/s from ${state.kneeAngleDeg.toFixed(1)}° to ${Math.round(state.setupTargetDeg || state.romWindowDeg[0])}°. Do NOT place your leg yet — please stay away!`);
      }
      if (setupBannerProgressWrap && setupBannerProgressFill && setupBannerProgressText) {
        setupBannerProgressWrap.hidden = false;
        if (isPark || isAutoHome) {
          const spanRef = Math.max(1, state.romWindowDeg[0] || 120);
          const ratio = clamp(1 - (state.kneeAngleDeg / spanRef), 0, 1);
          setupBannerProgressFill.style.width = `${(ratio * 100).toFixed(1)}%`;
          setupBannerProgressText.textContent = `${state.kneeAngleDeg.toFixed(1)}° → 0.0°`;
        } else {
          const targetRef = Math.max(1, state.setupTargetDeg || state.romWindowDeg[0] || 120);
          const ratio = clamp(state.kneeAngleDeg / targetRef, 0, 1);
          setupBannerProgressFill.style.width = `${(ratio * 100).toFixed(1)}%`;
          setupBannerProgressText.textContent = `${state.kneeAngleDeg.toFixed(1)}° → ${targetRef.toFixed(1)}°`;
        }
      }
      if (cancelSetupBtn) {
        cancelSetupBtn.hidden = false;
        cancelSetupBtn.textContent = "Stop & Release";
      }
      if (bannerHardStopBtn) bannerHardStopBtn.hidden = false;
      return;
    }

    if (state.setupState === "ready_at_start") {
      setupWarningBanner.hidden = false;
      setupWarningBanner.classList.add("is-ready");
      if (setupBannerBadge) setupBannerBadge.textContent = "READY FOR LEG";
      if (setupBannerTitle) {
        setupBannerTitle.textContent = `✓ Starting Position Reached (${Math.round(state.romWindowDeg[0])}°)`;
      }
      if (setupBannerMessage) {
        setupBannerMessage.textContent =
          "Arm is holding gently at the extended starting position. Place your leg securely on the shin pad, then click Begin Exercise.";
      }
      if (beginExerciseBtn) beginExerciseBtn.hidden = false;
      if (cancelSetupBtn) {
        cancelSetupBtn.hidden = false;
        cancelSetupBtn.textContent = "Cancel & Release";
      }
      return;
    }

    if (state.unhomedPrompt && state.espConnected && state.u2Linked && !state.u2Homed) {
      setupWarningBanner.hidden = false;
      setupWarningBanner.classList.add("is-unhomed");
      if (setupBannerBadge) setupBannerBadge.textContent = "HOMING REQUIRED";
      if (setupBannerTitle) setupBannerTitle.textContent = "Top-Hub Homing Required (H:0)";
      if (setupBannerMessage) {
        setupBannerMessage.textContent =
          "Home the Top-Hub at the 0° mechanical stop before starting therapy so the arm knows its exact angle and never overdrives.";
      }
      if (bannerQuickHomeBtn) bannerQuickHomeBtn.hidden = false;
      if (bannerAutoHomeBtn) bannerAutoHomeBtn.hidden = false;
      return;
    }

    setupWarningBanner.hidden = true;
  }

  function updateTopHubHardwareBar() {
    if (thLinkChip) {
      if (!state.espConnected) {
        thLinkChip.textContent = "Master WS: Offline (Demo)";
        thLinkChip.className = "clinical-flexion-chip";
      } else if (state.u2Linked) {
        thLinkChip.textContent = "Top-Hub U2: Linked (50 Hz)";
        thLinkChip.className = "clinical-flexion-chip ok";
      } else {
        thLinkChip.textContent = "Top-Hub U2: Scanning ESP-NOW…";
        thLinkChip.className = "clinical-flexion-chip warn";
      }
    }

    if (thHomeChip) {
      if (state.u2HardStop || state.estopped) {
        thHomeChip.textContent = "Safety: HARD-STOP LOCKED";
        thHomeChip.className = "clinical-flexion-chip warn";
      } else if (state.u2Spasm) {
        thHomeChip.textContent = "Safety: Spasm Guard Active";
        thHomeChip.className = "clinical-flexion-chip warn";
      } else if (state.u2FaultLockout) {
        thHomeChip.textContent = "Safety: Fault Lockout";
        thHomeChip.className = "clinical-flexion-chip warn";
      } else if (state.u2BusyHoming || state.setupState === "autohoming") {
        thHomeChip.textContent = "Home: Auto-Homing (Stay Away)…";
        thHomeChip.className = "clinical-flexion-chip warn";
      } else if (state.setupState === "parking_home") {
        thHomeChip.textContent = "Setup: Returning to 0° Park…";
        thHomeChip.className = "clinical-flexion-chip warn";
      } else if (state.u2Positioning || state.setupState === "moving_to_start" || state.setupWarning) {
        thHomeChip.textContent = "Setup: Positioning to 120°…";
        thHomeChip.className = "clinical-flexion-chip warn";
      } else if (state.setupState === "ready_at_start") {
        thHomeChip.textContent = "Setup: Ready at Start (Place Leg)";
        thHomeChip.className = "clinical-flexion-chip ok";
      } else if (state.u2Homed) {
        thHomeChip.textContent = "Home: Latch Ready (H:1)";
        thHomeChip.className = "clinical-flexion-chip ok";
      } else {
        thHomeChip.textContent = "Home: Unhomed (H:0)";
        thHomeChip.className = "clinical-flexion-chip";
      }
    }

    if (thAngleChip) {
      thAngleChip.textContent =
        `Knee Angle θk: ${state.kneeAngleDeg.toFixed(1)}° (${state.kneeVelDegS >= 0 ? "+" : ""}${state.kneeVelDegS.toFixed(1)}°/s)`;
    }

    if (thPowerChip) {
      thPowerChip.textContent = `Drive: ${state.vbusVolts.toFixed(1)} V · ${state.motorCurrentA.toFixed(2)} A`;
    }

    if (thWorkChip) {
      thWorkChip.textContent = `Last Rep Work: ${state.lastRepWorkJ.toFixed(2)} J`;
    }

    if (thAngleBarFill) {
      const pct = clamp((state.kneeAngleDeg / 180) * 100, 0, 100);
      thAngleBarFill.style.width = `${pct.toFixed(1)}%`;
    }

    if (hardStopBtn) {
      const locked = state.estopped || state.u2HardStop;
      hardStopBtn.textContent = locked ? "Release Hard Stop" : "Hard Stop";
      hardStopBtn.classList.toggle("engaged", locked);
    }

    if (pauseSessionBtn) {
      const inSetup = state.setupState === "moving_to_start" ||
        state.setupState === "ready_at_start" ||
        state.setupState === "parking_home" ||
        state.setupState === "autohoming";
      pauseSessionBtn.disabled = inSetup;
      if (state.setupState === "moving_to_start") {
        pauseSessionBtn.textContent = "Positioning Arm…";
      } else if (state.setupState === "ready_at_start") {
        pauseSessionBtn.textContent = "Click Begin Exercise Above";
      } else {
        pauseSessionBtn.textContent = (state.paused || state.estopped) ? "Resume Session" : "Pause Session";
      }
    }

    renderSafetyBanner();
  }

  function updateCountdownUI() {
    const remaining = Math.max(0, state.targetReps - state.completedReps);
    const pct = state.targetReps > 0
      ? clamp((state.completedReps / state.targetReps) * 100, 0, 100)
      : 0;

    if (repsRemainingValue) {
      repsRemainingValue.textContent = String(remaining);
      repsRemainingValue.classList.remove("bump");
      void repsRemainingValue.offsetWidth;
      repsRemainingValue.classList.add("bump");
    }
    if (repsRemainingCaption) {
      repsRemainingCaption.textContent = remaining === 1 ? "rep remaining" : "reps remaining";
    }
    if (repsProgressText) {
      repsProgressText.textContent = `${state.completedReps} / ${state.targetReps} completed`;
    }
    if (repProgressFill) {
      repProgressFill.style.width = `${pct.toFixed(1)}%`;
    }

    const done = remaining === 0 && state.active;
    if (sessionCompleteBanner) {
      sessionCompleteBanner.hidden = !done && !state.ended;
    }

    if (done && !state.ended) {
      state.ended = true;
      state.paused = true;
      if (bannerTitle) bannerTitle.textContent = "Session Complete";
      if (bannerMessage) {
        bannerMessage.textContent = `Completed ${state.completedReps} of ${state.targetReps} repetitions in ${currentMode().title}. Motor torque released — remove your leg before clicking Return to 0° Park.`;
      }
      setOdometerValue(0);
      sendEspCommand({ type: "stop_session", reason: "reps_complete" });
      persistCompletedSession("completed");
    }
  }

  function recordOneRep(sourceLabel) {
    if (!state.active || state.paused || state.ended || state.estopped) return;
    if (state.completedReps >= state.targetReps) return;

    state.completedReps += 1;
    updateCountdownUI();
    if (espCommandStatus && sourceLabel) {
      espCommandStatus.textContent = `${sourceLabel} (${state.completedReps}/${state.targetReps} reps)`;
    }
  }

  function updateConnectionUI(isLive, customText) {
    state.espConnected = Boolean(isLive);

    if (connectionBadge) {
      connectionBadge.className = `exercise-mode-indicator ${isLive ? "live" : "demo"}`;
      connectionBadge.textContent = customText || (isLive ? "● LIVE — ESP32 CONNECTED" : "● DEMO — NO ESP32");
    }
    if (espStatusPill) {
      espStatusPill.className = `pill-status ${isLive ? "live" : "demo"}`;
      espStatusPill.textContent = isLive
        ? (state.u2Linked ? "● Top-Hub U2 Linked" : "● Master Connected")
        : "● Demo Simulator";
    }
    if (torqueSourceTag) {
      torqueSourceTag.textContent = isLive
        ? (state.u2Linked ? "Top-Hub 50 Hz Stream" : "Master ESP Stream")
        : "Demo Telemetry";
    }
    if (repTestControls) {
      repTestControls.hidden = Boolean(isLive && state.u2Linked);
    }
    updateTopHubHardwareBar();
  }

  function buildSetParamsPayload(reason) {
    const mode = currentMode();
    return {
      type: "set_params",
      reason: reason || "ui_sync",
      mode: mode.key,
      op_mode: mode.opMode,
      role: mode.role,
      torque_nm: state.targetTorque,
      target_torque_ext: state.targetTorque,
      target_torque_flex: Number((state.targetTorque * 0.65).toFixed(1)),
      target_reps: state.targetReps,
      rom_min_deg: state.romWindowDeg[0],
      rom_max_deg: state.romWindowDeg[1],
      target_vel_deg_s: state.targetVelDegS,
      d_offset_m: state.dOffsetM,
      L_leg_m: state.lLegM,
      pathology_profile: state.pathologyCode
    };
  }

  function sendEspCommand(payload) {
    const message = JSON.stringify({
      ...payload,
      timestamp: Date.now()
    });

    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(message);
      return true;
    }
    if (socket && socket.readyState === WebSocket.CONNECTING) {
      pendingOpenCommands.push(message);
      return true;
    }
    return false;
  }

  function adjustTargetTorque(deltaSteps) {
    if (state.estopped) return;
    const mode = currentMode();
    const prev = state.targetTorque;
    const next = clamp(prev + deltaSteps * mode.step, mode.minTorque, mode.maxTorque);
    if (next === prev) return;

    state.targetTorque = next;
    state.lastLocalTorqueSetMs = Date.now();
    updateTargetTorqueUI();
    updateSetupSummary();

    const sent = sendEspCommand({
      type: "set_torque",
      mode: mode.key,
      op_mode: mode.opMode,
      role: mode.role,
      torque_nm: state.targetTorque,
      target_torque_ext: state.targetTorque,
      target_torque_flex: Number((state.targetTorque * 0.65).toFixed(1)),
      rom_min_deg: state.romWindowDeg[0],
      rom_max_deg: state.romWindowDeg[1],
      target_vel_deg_s: state.targetVelDegS
    });

    if (espCommandStatus) {
      espCommandStatus.textContent = sent
        ? `Sent ${mode.role} target ${state.targetTorque} N·m to Top-Hub via Master ESP32.`
        : `Set ${mode.role} target to ${state.targetTorque} N·m (Demo mode).`;
    }
  }

  function applyKinematicsFromInputs(sendNow, useActiveInputs) {
    const mode = currentMode();
    const minEl = useActiveInputs && activeRomMinInput ? activeRomMinInput : romMinInput;
    const maxEl = useActiveInputs && activeRomMaxInput ? activeRomMaxInput : romMaxInput;
    const rawMin = minEl ? Number(minEl.value) : state.romWindowDeg[0];
    const rawMax = maxEl ? Number(maxEl.value) : state.romWindowDeg[1];
    const rawVel = targetVelInput ? Number(targetVelInput.value) : state.targetVelDegS;

    const [rMin, rMax] = normalizeWorkingRom(rawMin, rawMax);
    const vel = mode.key === "isometric"
      ? clamp(Number.isFinite(rawVel) ? Math.round(rawVel) : 150, rMin, rMax)
      : clamp(Number.isFinite(rawVel) ? Math.round(rawVel) : mode.defaultVel, 5, 180);

    state.romWindowDeg = [rMin, rMax];
    state.targetVelDegS = vel;
    state.lastLocalKinSetMs = Date.now();
    syncKinematicsInputs();
    updateSetupSummary();
    updateTopHubHardwareBar();

    if (sendNow) {
      const sent = sendEspCommand(buildSetParamsPayload("kinematics_update"));
      if (topHubStatusNote) {
        topHubStatusNote.innerHTML = sent
          ? `Synced ROM <strong>${rMin}°–${rMax}°</strong> &amp; ${mode.key === "isometric" ? "hold" : "speed"} <strong>${vel}${mode.key === "isometric" ? "°" : "°/s"}</strong> to Master ESP32 → Top-Hub U2.`
          : `Updated ROM <strong>${rMin}°–${rMax}°</strong> &amp; ${mode.key === "isometric" ? "hold" : "speed"} <strong>${vel}${mode.key === "isometric" ? "°" : "°/s"}</strong> locally.`;
      }
      if (espCommandStatus && state.active) {
        espCommandStatus.textContent = sent
          ? `Updated live ROM to ${rMin}°–${rMax}° on Top-Hub U2.`
          : `Updated ROM to ${rMin}°–${rMax}° (Demo mode).`;
      }
    }
  }

  function updateFromESP(packet) {
    if (!packet || typeof packet !== "object") return;

    if (packet.type === "esp_status" || packet.type === "status") {
      if (packet.connected === false) {
        state.u2Linked = false;
        updateConnectionUI(false, "● DEMO — ESP32 OFFLINE");
        if (topHubStatusNote && packet.message) {
          topHubStatusNote.textContent = packet.message;
        }
      } else if (packet.connected === true) {
        updateConnectionUI(true, "● LIVE — ESP32 CONNECTED");
      }
      return;
    }

    if (packet.type === "esp_ack") {
      const msg = packet.message || (packet.forwarded === false ? "ESP32 not connected." : "Command acknowledged by Master ESP32.");
      if (packet.severity === "spasm" || packet.severity === "fault" || packet.severity === "warning") {
        state.lastSafetyMessage = msg;
      }
      if (topHubStatusNote) {
        topHubStatusNote.textContent = msg;
      }
      if (espCommandStatus) {
        espCommandStatus.textContent = msg;
      }
      if (packet.forwarded === false) {
        updateConnectionUI(false, "● DEMO — ESP32 OFFLINE");
      } else {
        renderSafetyBanner();
      }
      return;
    }

    // Parse all Top-Hub & setup state flags BEFORE updating badges
    if (packet.u2_linked !== undefined) {
      state.u2Linked = Number(packet.u2_linked) === 1;
    }
    if (packet.homed !== undefined) {
      state.u2Homed = Number(packet.homed) === 1;
      if (state.u2Homed) state.unhomedPrompt = false;
    }
    if (packet.calibrated !== undefined) {
      state.u2Calibrated = Number(packet.calibrated) === 1;
    }
    if (packet.busy_homing !== undefined) {
      state.u2BusyHoming = Number(packet.busy_homing) === 1;
    }
    if (packet.positioning !== undefined) {
      state.u2Positioning = Number(packet.positioning) === 1;
    }
    if (packet.hardstop_locked !== undefined) {
      state.u2HardStop = Number(packet.hardstop_locked) === 1;
    }
    if (packet.spasm !== undefined) {
      state.u2Spasm = Number(packet.spasm) === 1;
    }
    if (packet.fault_lockout !== undefined) {
      state.u2FaultLockout = Number(packet.fault_lockout) === 1;
    }
    if (typeof packet.setup_state === "string" && packet.setup_state.length > 0) {
      state.setupState = packet.setup_state;
    }
    if (packet.setup_warning !== undefined) {
      state.setupWarning = Number(packet.setup_warning) === 1;
    }
    if (Number.isFinite(Number(packet.setup_target_deg))) {
      state.setupTargetDeg = Number(packet.setup_target_deg);
    }
    if (Number.isFinite(Number(packet.knee_angle_deg))) {
      state.kneeAngleDeg = Number(packet.knee_angle_deg);
    }
    if (Number.isFinite(Number(packet.knee_vel_deg_s))) {
      state.kneeVelDegS = Number(packet.knee_vel_deg_s);
    }
    if (Number.isFinite(Number(packet.vbus_v))) {
      state.vbusVolts = Number(packet.vbus_v);
    }
    if (Number.isFinite(Number(packet.motor_current_a))) {
      state.motorCurrentA = Number(packet.motor_current_a);
    }
    if (Number.isFinite(Number(packet.last_rep_work_j))) {
      state.lastRepWorkJ = Number(packet.last_rep_work_j);
    }

    if (!state.u2Spasm && !state.u2FaultLockout && !state.u2HardStop && !state.estopped) {
      state.lastSafetyMessage = "";
    }

    const nowMs = Date.now();
    if (nowMs - state.lastLocalKinSetMs > 1200) {
      let kinUpdated = false;
      if (Number.isFinite(Number(packet.rom_min_deg)) && Number.isFinite(Number(packet.rom_max_deg))) {
        const pMin = Math.round(Number(packet.rom_min_deg));
        const pMax = Math.round(Number(packet.rom_max_deg));
        if (pMin >= 120 && pMax <= 180 && pMax >= pMin + 10 &&
            (pMin !== state.romWindowDeg[0] || pMax !== state.romWindowDeg[1])) {
          state.romWindowDeg = [pMin, pMax];
          kinUpdated = true;
        }
      }
      if (Number.isFinite(Number(packet.target_vel_deg_s)) && Number(packet.target_vel_deg_s) > 0) {
        const pVel = Math.round(Number(packet.target_vel_deg_s));
        if (pVel !== Math.round(state.targetVelDegS)) {
          state.targetVelDegS = pVel;
          kinUpdated = true;
        }
      }
      if (kinUpdated) {
        syncKinematicsInputs();
        updateSetupSummary();
      }
    }

    if (packet.estop !== undefined) {
      const nextEstop = Boolean(Number(packet.estop));
      if (nextEstop && !state.estopped) {
        state.estopped = true;
        state.paused = true;
        setOdometerValue(0);
        if (espCommandStatus) {
          espCommandStatus.textContent = "HARD STOP engaged on Top-Hub. Click Release Lock or Resume Session to unlock.";
        }
      } else if (!nextEstop && state.estopped) {
        state.estopped = false;
        if (espCommandStatus) {
          espCommandStatus.textContent = "Hard Stop released — Top-Hub unlocked.";
        }
      }
    }

    // Update connection indicator AFTER U2 link & safety state are current
    updateConnectionUI(true, state.u2Linked ? "● LIVE — TOP-HUB U2 LINKED" : "● LIVE — MASTER ESP32");

    if (!state.active || state.ended) return;

    if (packet.paused !== undefined) {
      state.paused = Boolean(Number(packet.paused));
    }
    if (state.u2Spasm || state.u2FaultLockout) {
      state.paused = true;
      setOdometerValue(0);
      if (espCommandStatus) {
        espCommandStatus.textContent = state.u2Spasm
          ? "Spasm guard triggered — motor released to zero-torque IDLE. Click Release Lock."
          : "Top-Hub safety fault — motor released to IDLE. Click Release Lock.";
      }
    }
    updateTopHubHardwareBar();

    const mode = currentMode();
    let incomingTorque = null;
    if (mode.role === "support" && Number.isFinite(Number(packet.support_torque))) {
      incomingTorque = Number(packet.support_torque);
    } else if (mode.role === "resistance" && Number.isFinite(Number(packet.resistance_torque))) {
      incomingTorque = Number(packet.resistance_torque);
    } else if (Number.isFinite(Number(packet.torque))) {
      incomingTorque = Number(packet.torque);
    }

    if (incomingTorque !== null) {
      setOdometerValue(incomingTorque);
    }

    if (Number.isFinite(Number(packet.target_torque)) && (nowMs - state.lastLocalTorqueSetMs > 900)) {
      state.targetTorque = clamp(Math.round(Number(packet.target_torque)), mode.minTorque, mode.maxTorque);
      updateTargetTorqueUI();
    }

    const inActiveTherapy = !state.paused && !state.estopped &&
      (state.setupState === "idle" || state.setupState === "");
    if (inActiveTherapy) {
      if (Number.isFinite(Number(packet.completed_reps))) {
        const nextCompleted = clamp(Math.round(Number(packet.completed_reps)), 0, state.targetReps);
        if (nextCompleted !== state.completedReps) {
          state.completedReps = nextCompleted;
          updateCountdownUI();
        }
      } else if (Number.isFinite(Number(packet.reps_remaining))) {
        const rem = clamp(Math.round(Number(packet.reps_remaining)), 0, state.targetReps);
        state.completedReps = state.targetReps - rem;
        updateCountdownUI();
      } else if (packet.rep_completed === true || packet.event === "rep") {
        recordOneRep("ESP rep sensor triggered");
      }
    }
  }

  window.updateFromESP = updateFromESP;

  function resolveWsUrl() {
    const params = new URLSearchParams(window.location.search);
    const espParam = params.get("esp");
    if (espParam) {
      return espParam.startsWith("ws://") || espParam.startsWith("wss://")
        ? espParam
        : `ws://${espParam}/`;
    }
    if (state.wsUrl) {
      return state.wsUrl;
    }
    if (window.location.host) {
      const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
      return `${proto}//${window.location.host}/ws/device`;
    }
    return null;
  }

  function connectEspSocket() {
    if (!state.shouldConnectWs) {
      updateConnectionUI(false);
      return;
    }
    const url = resolveWsUrl();
    if (!url) {
      updateConnectionUI(false);
      return;
    }

    if (socket && (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING)) {
      return;
    }

    clearTimeout(reconnectTimer);
    try {
      socket = new WebSocket(url);
    } catch (_err) {
      updateConnectionUI(false);
      return;
    }

    socket.onopen = () => {
      updateConnectionUI(true);
      while (pendingOpenCommands.length > 0) {
        const nextMsg = pendingOpenCommands.shift();
        socket.send(nextMsg);
      }
    };

    socket.onmessage = (event) => {
      try {
        const packet = JSON.parse(event.data);
        updateFromESP(packet);
      } catch (_err) {
        // Ignore non-JSON frames
      }
    };

    socket.onerror = () => {
      // Handled in onclose
    };

    socket.onclose = () => {
      updateConnectionUI(false);
      if (state.shouldConnectWs) {
        reconnectTimer = setTimeout(connectEspSocket, 3500);
      }
    };
  }

  function startDemoLoop() {
    clearInterval(simTimer);
    const tickMs = 120;

    simTimer = setInterval(() => {
      if (!state.active || state.paused || state.ended || state.estopped) return;
      if (state.espConnected && state.u2Linked) return;

      const mode = currentMode();
      state.simPhase += 0.24;

      const wave = Math.sin(state.simPhase) * mode.simAmplitude;
      const secondary = Math.cos(state.simPhase * 0.5) * (mode.simAmplitude * 0.45);
      const simulatedTorque = clamp(
        state.targetTorque + wave + secondary,
        mode.minTorque * 0.6,
        mode.maxTorque
      );
      setOdometerValue(simulatedTorque);

      const span = Math.max(15, state.romWindowDeg[1] - state.romWindowDeg[0]);
      const norm = 0.5 * (1 - Math.cos(state.simPhase));
      state.kneeAngleDeg = state.romWindowDeg[0] + span * norm;
      state.kneeVelDegS = Math.sin(state.simPhase) * state.targetVelDegS * 0.6;
      updateTopHubHardwareBar();

      if (state.autoSim && !state.espConnected) {
        state.simRepElapsedMs += tickMs;
        if (state.simRepElapsedMs >= 3600) {
          state.simRepElapsedMs = 0;
          recordOneRep("Auto demo rep completed");
        }
      }
    }, tickMs);
  }

  function buildStartSessionPayload() {
    const mode = currentMode();
    return {
      type: "start_session",
      mode: mode.key,
      op_mode: mode.opMode,
      role: mode.role,
      target_reps: state.targetReps,
      torque_nm: state.targetTorque,
      target_torque_ext: state.targetTorque,
      target_torque_flex: Number((state.targetTorque * 0.65).toFixed(1)),
      rom_min_deg: state.romWindowDeg[0],
      rom_max_deg: state.romWindowDeg[1],
      target_vel_deg_s: state.targetVelDegS,
      d_offset_m: state.dOffsetM,
      L_leg_m: state.lLegM,
      pathology_profile: state.pathologyCode
    };
  }

  function startSession() {
    applyKinematicsFromInputs(false, false);

    if (state.espConnected && state.u2Linked && !state.u2Homed) {
      state.unhomedPrompt = true;
      if (topHubStatusNote) {
        topHubStatusNote.textContent = "Top-Hub is unhomed (H:0). Use Set 0° Home or Auto-Home (STOP1) before starting therapy.";
      }
      renderSafetyBanner();
      window.scrollTo({ top: 0, behavior: "smooth" });
      return;
    }

    const mode = currentMode();
    const needsSlowSetup = Boolean(
      state.espConnected &&
      state.u2Linked &&
      (state.kneeAngleDeg < state.romWindowDeg[0] - 4 ||
       state.kneeAngleDeg > state.romWindowDeg[1] + 4)
    );

    state.unhomedPrompt = false;
    state.active = true;
    state.paused = needsSlowSetup;
    state.ended = false;
    state.estopped = false;
    state.u2Spasm = false;
    state.u2FaultLockout = false;
    state.lastSafetyMessage = "";
    state.setupState = needsSlowSetup ? "moving_to_start" : "idle";
    state.setupWarning = needsSlowSetup;
    state.setupTargetDeg = state.romWindowDeg[0];
    state.completedReps = 0;
    state.peakTorque = 0;
    state.torqueSum = 0;
    state.torqueSamples = 0;
    state.startedAtMs = Date.now();
    state.simPhase = 0;
    state.simRepElapsedMs = 0;
    state.sessionSaved = false;
    state.savingSession = false;
    state.lastSavePayload = null;

    if (!state.espConnected || !state.u2Linked) {
      state.kneeAngleDeg = state.romWindowDeg[0];
    }

    if (saveFeedback) saveFeedback.hidden = true;
    if (setupView) setupView.hidden = true;
    if (activeView) activeView.hidden = false;

    if (activeModePill) {
      activeModePill.textContent = mode.clinicalLabel;
    }
    if (activeSuggestedTag) {
      const sourcePrefix = state.clinicalRx?.flexion ? "Clinical Rx" : "Therapist suggested";
      activeSuggestedTag.textContent = `${sourcePrefix}: ${mode.suggestedReps} reps · ${state.romWindowDeg[0]}°–${state.romWindowDeg[1]}°`;
    }
    if (torqueRoleEyebrow) {
      torqueRoleEyebrow.textContent = mode.odometerTitle;
    }
    if (torqueOdometerLabel) {
      torqueOdometerLabel.textContent = mode.odometerSubLabel;
    }
    if (setpointLabel) {
      setpointLabel.textContent = mode.setpointLabel;
    }
    if (espCommandStatus) {
      espCommandStatus.textContent = needsSlowSetup
        ? `Setup in progress — moving arm slowly (10°/s) to ${state.romWindowDeg[0]}° start. Please stay away!`
        : `Active ${mode.title}: ${mode.role} target ${state.targetTorque} N·m (${state.romWindowDeg[0]}°–${state.romWindowDeg[1]}°).`;
    }
    if (activeTorqueRec) {
      if (state.clinicalRx?.flexion) {
        const fx = state.clinicalRx.flexion;
        const rxRom = Array.isArray(fx.machineRomWindowDeg)
          ? normalizeWorkingRom(fx.machineRomWindowDeg[0], fx.machineRomWindowDeg[1])
          : mapAnatomicalToMachineRom(fx.romWindowDeg?.[0], fx.romWindowDeg?.[1]);
        const rxTorque = clamp(Math.round(Number(fx.hardwareTorqueNm ?? fx.targetTorqueNm) || state.targetTorque), mode.minTorque, mode.maxTorque);
        activeTorqueRec.hidden = false;
        activeTorqueRec.textContent = `Recommended for your intake (${state.clinicalRx.phase}): ${rxTorque} N·m · ROM ${rxRom[0]}°–${rxRom[1]}°`;
      } else {
        activeTorqueRec.hidden = true;
      }
    }

    updateTargetTorqueUI();
    updateCountdownUI();
    setOdometerValue(needsSlowSetup ? 0 : mode.defaultTarget * 0.45);
    updateTopHubHardwareBar();

    if (state.shouldConnectWs) connectEspSocket();
    sendEspCommand(buildStartSessionPayload());
    startDemoLoop();
    window.scrollTo({ top: 0, behavior: "smooth" });
  }

  function exitToSetup() {
    if (state.active && !state.sessionSaved && state.completedReps > 0) {
      persistCompletedSession("ended_early");
    }
    state.active = false;
    state.paused = false;
    state.ended = false;
    state.setupState = "idle";
    state.setupWarning = false;
    clearInterval(simTimer);
    sendEspCommand({ type: "stop_session", reason: "return_to_setup" });

    if (activeView) activeView.hidden = true;
    if (setupView) setupView.hidden = false;
    updateTopHubHardwareBar();
    window.scrollTo({ top: 0, behavior: "smooth" });
  }

  function showSaveFeedback(message, isError) {
    if (!saveFeedback || !saveFeedbackText) return;
    saveFeedback.hidden = false;
    saveFeedback.className = `save-feedback${isError ? " error" : ""}`;
    saveFeedbackText.textContent = message;
    if (retrySaveBtn) {
      retrySaveBtn.hidden = !isError;
    }
  }

  async function persistCompletedSession(outcome) {
    if (state.sessionSaved || state.savingSession || state.completedReps < 1) return;
    const mode = currentMode();
    const durationSec = state.startedAtMs > 0
      ? Math.max(1, Math.round((Date.now() - state.startedAtMs) / 1000))
      : 1;
    const avgTorqueNm = state.torqueSamples > 0
      ? Number((state.torqueSum / state.torqueSamples).toFixed(1))
      : state.targetTorque;
    const peakTorqueNm = Number(Math.max(state.peakTorque, state.targetTorque).toFixed(1));
    const progressSource = (state.espConnected && state.u2Linked)
      ? "esp32"
      : (state.deviceMode === "connected" ? "connected-preview" : "demo");

    const payload = state.lastSavePayload || {
      type: mode.role === "support" ? "Support" : "Resistance",
      mode: mode.title,
      torqueNm: avgTorqueNm,
      reps: clamp(Math.round(state.completedReps), 1, 99),
      durationSeconds: durationSec,
      source: progressSource,
      outcome: outcome || "completed"
    };
    state.lastSavePayload = payload;
    state.savingSession = true;

    try {
      const res = await fetch("/api/progress", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload)
      });
      if (!res.ok) {
        throw new Error(`HTTP ${res.status}`);
      }
      state.sessionSaved = true;
      state.savingSession = false;
      showSaveFeedback(
        `Saved to your progress record: ${mode.title} · ${state.completedReps}/${state.targetReps} reps · Avg ${avgTorqueNm} N·m (Peak ${peakTorqueNm} N·m).`,
        false
      );
      await loadExerciseHistory();
    } catch (_err) {
      state.savingSession = false;
      showSaveFeedback(
        "Session finished, but saving to server failed. Click Retry Save to record it.",
        true
      );
    }
  }

  function applyClinicalPrescription(rx) {
    if (!rx || !rx.flexion) return;
    state.clinicalRx = rx;
    const fx = rx.flexion;
    const recKey = fx.modeId || fx.recommendedMode || "isotonic";

    const workingRom = Array.isArray(fx.machineRomWindowDeg) && fx.machineRomWindowDeg.length === 2
      ? normalizeWorkingRom(fx.machineRomWindowDeg[0], fx.machineRomWindowDeg[1])
      : (Array.isArray(fx.romWindowDeg) && fx.romWindowDeg.length === 2
        ? mapAnatomicalToMachineRom(fx.romWindowDeg[0], fx.romWindowDeg[1])
        : [120, 180]);
    state.romWindowDeg = workingRom;

    if (Number.isFinite(Number(fx.targetVelDegS)) && Number(fx.targetVelDegS) > 0) {
      state.targetVelDegS = Number(fx.targetVelDegS);
    }
    if (Number.isFinite(Number(fx.lLegM)) && Number(fx.lLegM) >= 0.15) {
      state.lLegM = Number(fx.lLegM);
    } else if (Number.isFinite(Number(rx.geometry?.lLegCm)) && Number(rx.geometry.lLegCm) >= 15) {
      state.lLegM = Number((Number(rx.geometry.lLegCm) / 100).toFixed(3));
    }
    if (Number.isFinite(Number(fx.pathologyCode))) {
      state.pathologyCode = Number(fx.pathologyCode);
    }

    if (MODE_CATALOG[recKey]) {
      const recMode = MODE_CATALOG[recKey];
      const hwTorque = Number(fx.hardwareTorqueNm ?? fx.targetTorqueNm);
      recMode.suggestedReps = clamp(Number(fx.suggestedReps) || recMode.suggestedReps, 1, 50);
      recMode.defaultTarget = clamp(Number.isFinite(hwTorque) ? Math.round(hwTorque) : recMode.defaultTarget, recMode.minTorque, recMode.maxTorque);
    }

    document.querySelectorAll("[data-rec-badge]").forEach((badge) => {
      const key = badge.getAttribute("data-rec-badge");
      badge.hidden = key !== recKey;
    });

    if (suggestionSourceLabel) {
      suggestionSourceLabel.textContent = "Clinical Intake Rx";
    }

    if (clinicalFlexionCard) {
      clinicalFlexionCard.hidden = false;
      const displayTorque = MODE_CATALOG[recKey]?.defaultTarget ?? Number(fx.hardwareTorqueNm ?? fx.targetTorqueNm ?? 22);
      clinicalFlexionCard.innerHTML = `
        <span class="clinical-strip-badge">★ Clinical Suggestion</span>
        <span><strong>${escapeHtml(fx.modeName)}</strong> · ${Number(fx.suggestedReps)} reps · ${displayTorque} N·m · ROM ${workingRom[0]}°–${workingRom[1]}°</span>
        <a class="clinical-strip-link" href="/intake">Edit Intake</a>
      `;
    }

    if (MODE_CATALOG[recKey]) {
      state.targetReps = MODE_CATALOG[recKey].suggestedReps;
      state.targetTorque = MODE_CATALOG[recKey].defaultTarget;
      selectMode(recKey, true);
    }
  }

  async function loadDeviceConfig() {
    const params = new URLSearchParams(window.location.search);
    const hasEspParam = Boolean(params.get("esp"));
    if (hasEspParam) state.shouldConnectWs = true;

    try {
      const [cfgRes, meRes] = await Promise.all([
        fetch("/api/device/config"),
        fetch("/api/me")
      ]);
      let rx = null;
      if (cfgRes.ok) {
        const cfg = await cfgRes.json();
        const wsPath = cfg.bridgeWsPath || cfg.wsProxyPath;
        if (wsPath && window.location.host) {
          const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
          state.wsUrl = `${proto}//${window.location.host}${wsPath}`;
        }
        const isConnected = cfg.deviceMode === "connected" || Boolean(cfg.connected);
        state.deviceMode = isConnected ? "connected" : (cfg.deviceMode || "demo");
        state.shouldConnectWs = isConnected || hasEspParam;
        if (isConnected) {
          updateConnectionUI(true, "● CONNECTED — WAITING STREAM");
        }
        if (cfg.prescription) {
          rx = cfg.prescription;
        }
      }
      if (meRes.ok) {
        const me = await meRes.json();
        if (!rx && me?.user?.prescription) {
          rx = me.user.prescription;
        }
      }
      if (rx) {
        applyClinicalPrescription(rx);
      }
    } catch (_err) {
      // Static file fallback
    }
    if (state.shouldConnectWs) {
      connectEspSocket();
    }
  }

  async function loadExerciseHistory() {
    if (!historyList) return;
    try {
      const res = await fetch("/api/progress");
      if (!res.ok) throw new Error("Unauthenticated or offline");
      const data = await res.json();
      const history = Array.isArray(data.sessions) ? data.sessions : [];
      if (!history.length) {
        historyList.innerHTML = `<div class="exercise-history-empty">No exercises recorded yet. Complete a session above to log your progress.</div>`;
        return;
      }
      historyList.innerHTML = history.slice(0, 6).map((item) => {
        const when = item.date ? new Date(item.date).toLocaleString() : "Recorded session";
        const durLabel = item.durationSeconds ? `${Math.round(item.durationSeconds)}s` : `${item.reps} reps`;
        const highlight = item.torqueNm != null ? `${item.torqueNm} N·m` : `${item.reps} reps`;
        return `
        <div class="exercise-history-row">
          <div>
            <strong>${escapeHtml(item.mode || item.type || "Exercise")}</strong>
            <div class="exercise-history-meta">${escapeHtml(when)} · ${escapeHtml(String(item.reps || 0))} reps</div>
          </div>
          <div class="exercise-history-stats">
            <span class="exercise-stat-pill">${escapeHtml(durLabel)}</span>
            <span class="exercise-stat-pill">${escapeHtml(highlight)}</span>
          </div>
        </div>
      `;
      }).join("");
    } catch (_err) {
      historyList.innerHTML = `
        <div class="exercise-history-row">
          <div>
            <strong>Flexion &amp; Extension (Isotonic)</strong>
            <div class="exercise-history-meta">Recent clinical benchmark</div>
          </div>
          <div class="exercise-history-stats">
            <span class="exercise-stat-pill">12/12 reps</span>
            <span class="exercise-stat-pill">22 N·m target</span>
          </div>
        </div>
      `;
    }
  }

  // Event Listeners
  if (menuToggleBtn && mobileNavDrawer) {
    menuToggleBtn.addEventListener("click", () => {
      const isOpen = mobileNavDrawer.classList.toggle("open");
      menuToggleBtn.setAttribute("aria-expanded", isOpen ? "true" : "false");
    });
  }

  function triggerQuickHome() {
    if (state.shouldConnectWs) connectEspSocket();
    state.unhomedPrompt = false;
    const sent = sendEspCommand({ type: "setup_home" });
    if (topHubStatusNote) {
      topHubStatusNote.innerHTML = sent
        ? "Sent <strong>Set 0° Home</strong> (<code>OP_SETUP_QUICK_6S</code>) to Master ESP32 → latching 0.0° &amp; ACS712 zero on Top-Hub U2…"
        : "Demo mode: simulated Top-Hub 0.0° Home latch (H:1).";
    }
    if (!sent) {
      state.u2Homed = true;
      state.kneeAngleDeg = 0;
      updateTopHubHardwareBar();
    } else {
      renderSafetyBanner();
    }
  }

  function triggerAutoHome() {
    if (state.shouldConnectWs) connectEspSocket();
    state.unhomedPrompt = false;
    state.setupState = "autohoming";
    state.setupWarning = true;
    state.setupTargetDeg = 0;
    const sent = sendEspCommand({ type: "setup_full" });
    if (topHubStatusNote) {
      topHubStatusNote.innerHTML = sent
        ? "Setup in progress — sent <strong>Auto-Home (STOP1)</strong> to Top-Hub U2. Please stay away!"
        : "Demo mode: simulated Top-Hub motorized Auto-Home (H:1).";
    }
    if (!sent) {
      state.setupState = "idle";
      state.setupWarning = false;
      state.u2Homed = true;
      state.kneeAngleDeg = 0;
    }
    updateTopHubHardwareBar();
  }

  function triggerParkHome() {
    if (state.shouldConnectWs) connectEspSocket();
    if (state.espConnected && state.u2Linked && !state.u2Homed) {
      state.unhomedPrompt = true;
      if (topHubStatusNote) {
        topHubStatusNote.textContent = "Top-Hub is unhomed (H:0) — use Auto-Home (STOP1) to seek 0° safely.";
      }
      renderSafetyBanner();
      return;
    }
    state.active = false;
    state.paused = true;
    state.setupState = "parking_home";
    state.setupWarning = true;
    state.setupTargetDeg = 0;
    clearInterval(simTimer);
    const sent = sendEspCommand({ type: "park_home" });
    if (topHubStatusNote) {
      topHubStatusNote.innerHTML = sent
        ? "Returning arm slowly (10°/s) to <strong>0° Park</strong> — please stay away!"
        : "Demo mode: arm returned smoothly to 0° Park.";
    }
    if (!sent) {
      state.setupState = "idle";
      state.setupWarning = false;
      state.kneeAngleDeg = 0;
      state.kneeVelDegS = 0;
    }
    updateTopHubHardwareBar();
    window.scrollTo({ top: 0, behavior: "smooth" });
  }

  function triggerReleaseLock() {
    if (state.shouldConnectWs) connectEspSocket();
    state.estopped = false;
    state.u2HardStop = false;
    state.u2Spasm = false;
    state.u2FaultLockout = false;
    state.lastSafetyMessage = "";
    const sent = sendEspCommand({ type: "hardstop_release" });
    updateTargetTorqueUI();
    updateTopHubHardwareBar();
    if (topHubStatusNote) {
      topHubStatusNote.textContent = sent
        ? "Sent Hard-Stop Release (ESTOP_HARD_STOP_RELEASE) to Master ESP32 → Top-Hub U2 unlocked."
        : "Cleared Hard-Stop & spasm lock state locally.";
    }
    if (espCommandStatus) {
      espCommandStatus.textContent = sent
        ? "Safety lock released — click Resume Session or Return to 0° Park."
        : "Safety lock cleared locally.";
    }
  }

  function engageHardStop() {
    state.estopped = true;
    state.paused = true;
    state.setupState = "idle";
    state.setupWarning = false;
    setOdometerValue(0);
    updateTargetTorqueUI();
    updateTopHubHardwareBar();
    sendEspCommand({ type: "estop", estop_active: 1 });
    if (espCommandStatus) {
      espCommandStatus.textContent = "HARD STOP engaged — motor torque set to 0 N·m. Click Release Lock or Resume Session to unlock.";
    }
  }

  if (quickHomeBtn) {
    quickHomeBtn.addEventListener("click", triggerQuickHome);
  }

  if (autoHomeBtn) {
    autoHomeBtn.addEventListener("click", triggerAutoHome);
  }

  if (parkHomeBtn) {
    parkHomeBtn.addEventListener("click", triggerParkHome);
  }

  if (completeParkBtn) {
    completeParkBtn.addEventListener("click", triggerParkHome);
  }

  if (releaseLockBtn) {
    releaseLockBtn.addEventListener("click", triggerReleaseLock);
  }

  if (bannerQuickHomeBtn) {
    bannerQuickHomeBtn.addEventListener("click", triggerQuickHome);
  }

  if (bannerAutoHomeBtn) {
    bannerAutoHomeBtn.addEventListener("click", triggerAutoHome);
  }

  if (bannerReleaseBtn) {
    bannerReleaseBtn.addEventListener("click", triggerReleaseLock);
  }

  if (bannerHardStopBtn) {
    bannerHardStopBtn.addEventListener("click", engageHardStop);
  }

  if (beginExerciseBtn) {
    beginExerciseBtn.addEventListener("click", () => {
      state.setupState = "idle";
      state.setupWarning = false;
      state.paused = false;
      sendEspCommand({ type: "begin_exercise" });
      if (espCommandStatus) {
        espCommandStatus.textContent = `Exercise active in ${currentMode().title} (${state.romWindowDeg[0]}°–${state.romWindowDeg[1]}°).`;
      }
      updateTopHubHardwareBar();
    });
  }

  if (cancelSetupBtn) {
    cancelSetupBtn.addEventListener("click", () => {
      exitToSetup();
    });
  }

  modeCards.forEach((card) => {
    card.addEventListener("click", () => {
      const modeKey = card.getAttribute("data-mode");
      selectMode(modeKey);
    });
  });

  extraModePills.forEach((pill) => {
    pill.addEventListener("click", () => {
      const modeKey = pill.getAttribute("data-mode");
      selectMode(modeKey);
    });
  });

  romPresetBtns.forEach((btn) => {
    btn.addEventListener("click", () => {
      const [rMin, rMax] = normalizeWorkingRom(btn.getAttribute("data-rom-min"), btn.getAttribute("data-rom-max"));
      if (romMinInput) romMinInput.value = String(rMin);
      if (romMaxInput) romMaxInput.value = String(rMax);
      applyKinematicsFromInputs(true, false);
    });
  });

  if (romMinInput) {
    romMinInput.addEventListener("change", () => applyKinematicsFromInputs(true, false));
  }
  if (romMaxInput) {
    romMaxInput.addEventListener("change", () => applyKinematicsFromInputs(true, false));
  }
  if (activeRomMinInput) {
    activeRomMinInput.addEventListener("change", () => applyKinematicsFromInputs(true, true));
  }
  if (activeRomMaxInput) {
    activeRomMaxInput.addEventListener("change", () => applyKinematicsFromInputs(true, true));
  }
  if (applyActiveRomBtn) {
    applyActiveRomBtn.addEventListener("click", () => applyKinematicsFromInputs(true, true));
  }
  if (targetVelInput) {
    targetVelInput.addEventListener("change", () => applyKinematicsFromInputs(true, false));
  }
  if (applyKinematicsBtn) {
    applyKinematicsBtn.addEventListener("click", () => applyKinematicsFromInputs(true, false));
  }

  if (useSuggestedBtn) {
    useSuggestedBtn.addEventListener("click", () => {
      setTargetReps(currentMode().suggestedReps);
    });
  }

  if (repMinusBtn) {
    repMinusBtn.addEventListener("click", () => {
      setTargetReps(state.targetReps - 1);
    });
  }

  if (repPlusBtn) {
    repPlusBtn.addEventListener("click", () => {
      setTargetReps(state.targetReps + 1);
    });
  }

  if (startWorkoutBtn) {
    startWorkoutBtn.addEventListener("click", startSession);
  }

  if (backToSetupBtn) {
    backToSetupBtn.addEventListener("click", exitToSetup);
  }

  if (chooseAnotherModeBtn) {
    chooseAnotherModeBtn.addEventListener("click", exitToSetup);
  }

  if (repeatSessionBtn) {
    repeatSessionBtn.addEventListener("click", startSession);
  }

  if (retrySaveBtn) {
    retrySaveBtn.addEventListener("click", () => {
      persistCompletedSession("completed");
    });
  }

  if (torqueMinusBtn) {
    torqueMinusBtn.addEventListener("click", () => adjustTargetTorque(-1));
  }

  if (torquePlusBtn) {
    torquePlusBtn.addEventListener("click", () => adjustTargetTorque(1));
  }

  if (simRepBtn) {
    simRepBtn.addEventListener("click", () => {
      recordOneRep("Manual test rep logged");
    });
  }

  if (autoSimToggleBtn) {
    autoSimToggleBtn.addEventListener("click", () => {
      state.autoSim = !state.autoSim;
      autoSimToggleBtn.textContent = state.autoSim ? "Pause Auto Demo" : "Resume Auto Demo";
    });
  }

  if (pauseSessionBtn) {
    pauseSessionBtn.addEventListener("click", () => {
      if (!state.active || state.ended) return;
      if (state.setupState === "moving_to_start" || state.setupState === "ready_at_start") return;
      if (state.estopped || state.u2HardStop || state.u2Spasm || state.u2FaultLockout) {
        state.estopped = false;
        state.u2HardStop = false;
        state.u2Spasm = false;
        state.u2FaultLockout = false;
        state.lastSafetyMessage = "";
        state.paused = false;
        sendEspCommand({ type: "resume_session" });
        updateTargetTorqueUI();
        updateTopHubHardwareBar();
        if (espCommandStatus) {
          espCommandStatus.textContent = "Safety lock cleared & session resumed.";
        }
        return;
      }
      state.paused = !state.paused;
      sendEspCommand({
        type: state.paused ? "pause_session" : "resume_session"
      });
      updateTopHubHardwareBar();
      if (espCommandStatus) {
        espCommandStatus.textContent = state.paused ? "Session paused." : "Session resumed.";
      }
    });
  }

  if (endSessionBtn) {
    endSessionBtn.addEventListener("click", exitToSetup);
  }

  if (hardStopBtn) {
    hardStopBtn.addEventListener("click", () => {
      if (state.estopped || state.u2HardStop) {
        triggerReleaseLock();
        return;
      }
      engageHardStop();
    });
  }

  // Initialize UI
  buildOdometerTicks();
  selectMode("isotonic");
  updateConnectionUI(false);
  updateTopHubHardwareBar();
  loadDeviceConfig();
  loadExerciseHistory();
})();
