const MODES = {
  passive: {
    id: 'passive',
    name: 'Passive Assist Mode',
    shortName: 'Passive Assist',
    progressType: 'Support',
    role: 'support',
    roleLabel: 'Support Torque',
    eyebrow: 'Support Torque · ESP',
    setpointLabel: 'Support Target → ESP',
    opMode: 3,
    suggestedReps: 15,
    defaultTorque: 12,
    minTorque: 4,
    maxTorque: 25,
    stepTorque: 1,
  },
  isotonic: {
    id: 'isotonic',
    name: 'Physiological Curve-Shaped Isotonic',
    shortName: 'Isotonic Mode',
    progressType: 'Resistance',
    role: 'resistance',
    roleLabel: 'Resistance Torque',
    eyebrow: 'Resistance Torque · ESP',
    setpointLabel: 'Resistance Target → ESP',
    opMode: 4,
    suggestedReps: 12,
    defaultTorque: 25,
    minTorque: 5,
    maxTorque: 68,
    stepTorque: 1,
  },
  isokinetic: {
    id: 'isokinetic',
    name: 'Isokinetic Mode',
    shortName: 'Isokinetic Mode',
    progressType: 'Resistance',
    role: 'resistance',
    roleLabel: 'Resistance Torque',
    eyebrow: 'Resistance Torque · ESP',
    setpointLabel: 'Resistance Cap → ESP',
    opMode: 5,
    suggestedReps: 10,
    defaultTorque: 35,
    minTorque: 10,
    maxTorque: 68,
    stepTorque: 1,
  },
}

const PRESET_REPS = [8, 10, 12, 15, 20]
const MIN_REPS = 1
const MAX_REPS = 99
const ARC_CX = 130
const ARC_CY = 125
const ARC_RADIUS = 92

const boltIcon = '<svg aria-hidden="true" viewBox="0 0 32 32" fill="none" stroke="currentColor"><path d="M18.5 2.5 8 17h7l-1.5 12.5L24 14.5h-7l1.5-12Z" stroke-width="1.5" stroke-linejoin="round"/></svg>'
const cycleIcon = '<svg aria-hidden="true" viewBox="0 0 32 32" fill="none" stroke="currentColor"><circle cx="9" cy="22" r="5.5" stroke-width="1.5"/><circle cx="24" cy="22" r="5.5" stroke-width="1.5"/><path d="m9 22 5-10 4 10m-9 0h9l-5-7h8M13 9h4" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/></svg>'

const $ = (id) => document.getElementById(id)
const clamp = (value, min, max) => Math.min(max, Math.max(min, value))
const easeOutCubic = (t) => 1 - Math.pow(1 - t, 3)
const escapeHtml = (value) => String(value ?? '').replace(/[&<>"']/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch]))

const state = {
  mode: 'isotonic',
  targetReps: MODES.isotonic.suggestedReps,
  completedReps: 0,
  targetTorque: MODES.isotonic.defaultTorque,
  liveTorque: 0,
  visualTorque: 0,
  paused: false,
  estopped: false,
  active: false,
  ended: false,
  saving: false,
  saved: false,
  deviceMode: 'demo',
  bypassEsp: false,
  wsUrl: '',
  wsConnected: false,
  demoRunning: true,
  startedAt: 0,
  pausedAt: 0,
  pausedMs: 0,
  sessions: [],
}

let socket = null
let simTimer = null
let autoRepAccumulator = 0
let animFromTorque = 0
let animToTorque = 0
let animStartedAt = 0
let animFrame = null

function buildOdometerTicks() {
  const group = $('odoTicks')
  const ns = 'http://www.w3.org/2000/svg'
  const total = 32
  for (let i = 0; i <= total; i += 1) {
    const angle = (135 + (270 * i) / total) * (Math.PI / 180)
    const isMajor = i % 4 === 0
    const r1 = ARC_RADIUS - (isMajor ? 16 : 12)
    const r2 = ARC_RADIUS - 7
    const line = document.createElementNS(ns, 'line')
    line.setAttribute('x1', (ARC_CX + Math.cos(angle) * r1).toFixed(2))
    line.setAttribute('y1', (ARC_CY + Math.sin(angle) * r1).toFixed(2))
    line.setAttribute('x2', (ARC_CX + Math.cos(angle) * r2).toFixed(2))
    line.setAttribute('y2', (ARC_CY + Math.sin(angle) * r2).toFixed(2))
    if (isMajor) line.classList.add('major')
    group.appendChild(line)
  }
}

function activeModeConfig() {
  return MODES[state.mode]
}

function formatDate(value) {
  if (!value) return ''
  const date = new Date(value)
  return Number.isNaN(date.getTime())
    ? escapeHtml(value)
    : escapeHtml(date.toLocaleString('en-IN', { day: 'numeric', month: 'short', year: 'numeric', hour: 'numeric', minute: '2-digit' }))
}

function renderHistory() {
  const list = $('historyList')
  if (!list) return
  if (!state.sessions.length) {
    list.innerHTML = '<div class="exercise-history-empty">No sessions recorded yet. Complete or end a session to save it here.</div>'
    return
  }
  list.innerHTML = state.sessions.slice(0, 6).map((row) => {
    const modeDetail = row.mode && row.mode !== row.type ? ` · ${escapeHtml(row.mode)}` : ''
    const torqueDetail = Number.isFinite(Number(row.torqueNm)) ? ` · ${Number(row.torqueNm)} N·m` : ''
    return `<article class="exercise-history-row">
      <div class="exercise-history-icon">${row.type === 'Cycling' ? cycleIcon : boltIcon}</div>
      <div class="exercise-history-info">
        <strong>${escapeHtml(row.type)}${modeDetail}</strong>
        <small>${formatDate(row.date)}${torqueDetail} · ${row.source === 'demo' ? 'Demo' : 'Connected preview'}</small>
      </div>
      <div class="exercise-history-count">
        <strong>${Number(row.reps)}</strong>
        <small>reps</small>
      </div>
    </article>`
  }).join('')
}

async function loadHistory() {
  try {
    const res = await fetch('/api/progress', { credentials: 'same-origin' })
    if (!res.ok) return
    const data = await res.json()
    if (data.ok && Array.isArray(data.sessions)) {
      state.sessions = data.sessions
      renderHistory()
    }
  } catch {}
}

function updateSetupUI() {
  const modeCfg = activeModeConfig()

  document.querySelectorAll('.mode-card').forEach((card) => {
    const selected = card.dataset.mode === state.mode
    card.classList.toggle('selected', selected)
    card.setAttribute('aria-checked', String(selected))
  })

  $('suggestedModeName').textContent = modeCfg.shortName
  $('suggestedRepCount').textContent = `${modeCfg.suggestedReps} reps`
  $('repTargetDisplay').textContent = String(state.targetReps)
  $('repMinusBtn').disabled = state.targetReps <= MIN_REPS
  $('repPlusBtn').disabled = state.targetReps >= MAX_REPS

  const presetRoot = $('repPresets')
  presetRoot.innerHTML = ''
  for (const count of PRESET_REPS) {
    const btn = document.createElement('button')
    btn.type = 'button'
    btn.className = 'rep-preset-btn'
    if (count === state.targetReps) btn.classList.add('active')
    if (count === modeCfg.suggestedReps) btn.classList.add('is-suggested')
    btn.textContent = `${count}`
    btn.addEventListener('click', () => {
      state.targetReps = count
      updateSetupUI()
    })
    presetRoot.appendChild(btn)
  }

  $('setupSummary').innerHTML = `<strong>${modeCfg.name}</strong> · ${state.targetReps} reps · Initial ${modeCfg.roleLabel.toLowerCase()}: <strong>${state.targetTorque} N·m</strong>`
}

function paintTorqueOdometer(torqueNm) {
  const modeCfg = activeModeConfig()
  const clamped = clamp(torqueNm, 0, modeCfg.maxTorque)
  $('torqueValue').textContent = clamped.toFixed(1)
  const fraction = clamp(clamped / modeCfg.maxTorque, 0, 1)
  $('odoProgress').style.strokeDashoffset = (1000 * (1 - fraction)).toFixed(1)
}

function paintTargetMarker() {
  const modeCfg = activeModeConfig()
  const fraction = clamp(state.targetTorque / modeCfg.maxTorque, 0, 1)
  const angle = (135 + 270 * fraction) * (Math.PI / 180)
  const marker = $('odoTargetMarker')
  marker.setAttribute('cx', (ARC_CX + Math.cos(angle) * ARC_RADIUS).toFixed(2))
  marker.setAttribute('cy', (ARC_CY + Math.sin(angle) * ARC_RADIUS).toFixed(2))
}

function animateTorqueLoop(now) {
  const progress = clamp((now - animStartedAt) / 280, 0, 1)
  const eased = easeOutCubic(progress)
  state.visualTorque = animFromTorque + (animToTorque - animFromTorque) * eased
  paintTorqueOdometer(state.visualTorque)
  if (progress < 1) {
    animFrame = requestAnimationFrame(animateTorqueLoop)
  } else {
    state.visualTorque = animToTorque
    paintTorqueOdometer(state.visualTorque)
    animFrame = null
  }
}

function setLiveTorque(nextTorque) {
  const modeCfg = activeModeConfig()
  const safe = clamp(Number(nextTorque), 0, modeCfg.maxTorque)
  if (!Number.isFinite(safe)) return
  state.liveTorque = safe
  animFromTorque = state.visualTorque
  animToTorque = safe
  animStartedAt = performance.now()
  if (animFrame === null) animFrame = requestAnimationFrame(animateTorqueLoop)
}

function updateCountdownUI() {
  const remaining = Math.max(0, state.targetReps - state.completedReps)
  const pct = state.targetReps > 0 ? clamp((state.completedReps / state.targetReps) * 100, 0, 100) : 100
  $('repsRemainingValue').textContent = String(remaining)
  $('repsRemainingCaption').textContent = remaining === 1 ? 'rep remaining' : 'reps remaining'
  $('repsProgressText').textContent = `${state.completedReps} / ${state.targetReps} completed`
  $('repProgressFill').style.width = `${pct.toFixed(1)}%`

  const finished = remaining === 0 || state.ended
  $('sessionCompleteBanner').hidden = !finished
  $('simRepBtn').disabled = finished

  if (remaining === 0 && !state.ended) {
    $('bannerTitle').textContent = 'Session Complete'
    $('bannerMessage').textContent = 'All target repetitions have been completed safely.'
    if (!state.saved && !state.saving) {
      stopDemoTelemetry()
      void saveSessionProgress(state.targetReps)
    }
  } else if (state.ended) {
    $('bannerTitle').textContent = 'Session Ended'
    $('bannerMessage').textContent = `Logged ${Math.max(1, state.completedReps)} repetition(s) for ${activeModeConfig().name}.`
  }
}

function updateTorqueControlsUI() {
  const modeCfg = activeModeConfig()
  $('targetTorqueValue').textContent = `${state.targetTorque} N·m`
  $('torqueMinusBtn').disabled = state.targetTorque <= modeCfg.minTorque
  $('torquePlusBtn').disabled = state.targetTorque >= modeCfg.maxTorque
  paintTargetMarker()
}

function updateEspStatusPill() {
  const pill = $('espStatusPill')
  const sourceTag = $('torqueSourceTag')
  const testControls = $('repTestControls')
  const topBadge = $('connectionBadge')

  const isConnected = state.wsConnected || state.deviceMode === 'connected'
  topBadge.className = `exercise-mode-indicator ${isConnected ? 'connected' : 'demo'}`
  topBadge.textContent = isConnected ? '● ESP32 RESPONDED' : '● DEMO — NO ESP32'

  pill.classList.remove('warn', 'error')
  if (state.estopped) {
    pill.textContent = '● Hard Stop Engaged'
    pill.classList.add('error')
    sourceTag.textContent = 'Hard Stop Active'
    return
  }

  if (state.paused && !state.ended) {
    pill.textContent = '● Paused'
    pill.classList.add('warn')
    sourceTag.textContent = 'Session Paused'
  } else if (state.wsConnected) {
    pill.textContent = '● Live ESP32 Telemetry'
    sourceTag.textContent = 'From ESP WebSocket'
    testControls.hidden = true
  } else if (state.deviceMode === 'demo' || state.bypassEsp || !state.wsUrl) {
    pill.textContent = state.deviceMode === 'connected' ? '● Connected Preview' : '● Demo Simulation'
    pill.classList.add('warn')
    sourceTag.textContent = state.demoRunning ? 'Simulated ESP Stream' : 'Demo Paused'
    testControls.hidden = false
  } else {
    pill.textContent = '● Connecting to ESP32…'
    pill.classList.add('warn')
    sourceTag.textContent = 'Awaiting ESP Packets'
    testControls.hidden = false
  }
}

function sendEspCommand(payload) {
  const serialized = JSON.stringify(payload)
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(serialized)
    return { sentLive: true }
  }
  return { sentLive: false }
}

function adjustTorque(delta) {
  const modeCfg = activeModeConfig()
  const next = clamp(state.targetTorque + delta, modeCfg.minTorque, modeCfg.maxTorque)
  if (next === state.targetTorque) return
  state.targetTorque = next
  updateTorqueControlsUI()

  const payload = {
    type: 'set_torque',
    mode: modeCfg.id,
    op_mode: modeCfg.opMode,
    role: modeCfg.role,
    torque_nm: next,
    target_torque_ext: next,
    delta,
  }
  const { sentLive } = sendEspCommand(payload)
  $('espCommandStatus').textContent = sentLive
    ? `Sent to ESP: ${next} N·m (${modeCfg.roleLabel})`
    : `Target updated to ${next} N·m — ready to transmit to ESP`
}

function updateFromESP(packet) {
  if (!packet || typeof packet !== 'object' || !state.active || state.paused || state.ended) return

  const modeCfg = activeModeConfig()
  const incomingTorque = modeCfg.role === 'support'
    ? (packet.support_torque ?? packet.supportTorque ?? packet.torque ?? packet.knee_torque_nm ?? packet.target_curve_nm)
    : (packet.resistance_torque ?? packet.resistanceTorque ?? packet.torque ?? packet.knee_torque_nm ?? packet.target_curve_nm)

  if (incomingTorque !== undefined && Number.isFinite(Number(incomingTorque))) {
    setLiveTorque(Math.abs(Number(incomingTorque)))
  }

  if (packet.reps_remaining !== undefined && Number.isFinite(Number(packet.reps_remaining))) {
    const remaining = clamp(Math.round(Number(packet.reps_remaining)), 0, state.targetReps)
    state.completedReps = state.targetReps - remaining
    updateCountdownUI()
  } else {
    const repCount = packet.current_rep ?? packet.completed_reps ?? packet.completedReps ?? packet.rep_index ?? packet.rep
    if (repCount !== undefined && Number.isFinite(Number(repCount))) {
      state.completedReps = clamp(Math.round(Number(repCount)), 0, state.targetReps)
      updateCountdownUI()
    }
  }
}

function stopDemoTelemetry() {
  if (simTimer !== null) {
    clearInterval(simTimer)
    simTimer = null
  }
}

function startDemoTelemetry() {
  stopDemoTelemetry()
  if (!state.active || state.wsConnected || !state.demoRunning || state.ended) return

  simTimer = setInterval(() => {
    if (!state.active || state.paused || state.estopped || state.ended || state.wsConnected || !state.demoRunning) return
    const modeCfg = activeModeConfig()
    const remaining = state.targetReps - state.completedReps
    if (remaining <= 0) {
      setLiveTorque(0)
      return
    }
    const ripple = (Math.random() - 0.5) * (modeCfg.role === 'support' ? 1.4 : 2.6)
    setLiveTorque(clamp(state.targetTorque + ripple, 0, modeCfg.maxTorque))

    autoRepAccumulator += 300
    if (autoRepAccumulator >= 3600) {
      autoRepAccumulator = 0
      state.completedReps = Math.min(state.targetReps, state.completedReps + 1)
      updateCountdownUI()
    }
  }, 300)
}

function closeEspSocket() {
  if (socket) {
    try { socket.close() } catch {}
    socket = null
  }
  state.wsConnected = false
}

function connectEspSocket() {
  closeEspSocket()
  if (!state.wsUrl || !/^wss?:\/\//.test(state.wsUrl)) {
    updateEspStatusPill()
    startDemoTelemetry()
    return
  }

  try {
    socket = new WebSocket(state.wsUrl)
    socket.addEventListener('open', () => {
      state.wsConnected = true
      stopDemoTelemetry()
      updateEspStatusPill()
      const modeCfg = activeModeConfig()
      sendEspCommand({
        type: 'start_session',
        mode: modeCfg.id,
        op_mode: modeCfg.opMode,
        target_reps: state.targetReps,
        torque_nm: state.targetTorque,
      })
    })
    socket.addEventListener('message', (event) => {
      try {
        updateFromESP(JSON.parse(event.data))
      } catch (err) {
        console.warn('Invalid ESP32 JSON packet', err)
      }
    })
    socket.addEventListener('close', () => {
      state.wsConnected = false
      updateEspStatusPill()
      if (state.active && state.demoRunning && !state.ended) startDemoTelemetry()
    })
    socket.addEventListener('error', () => {
      state.wsConnected = false
      updateEspStatusPill()
    })
  } catch {
    state.wsConnected = false
    updateEspStatusPill()
    startDemoTelemetry()
  }
}

async function saveSessionProgress(repsOverride) {
  if (state.saving || state.saved) return
  state.saving = true

  const feedback = $('saveFeedback')
  const textEl = $('saveFeedbackText')
  const retryBtn = $('retrySaveBtn')
  feedback.hidden = false
  feedback.classList.remove('error')
  textEl.textContent = 'Saving session to Track Progress…'
  retryBtn.hidden = true

  const modeCfg = activeModeConfig()
  const activePausedMs = state.pausedMs + (state.paused && state.pausedAt ? Date.now() - state.pausedAt : 0)
  const durationSeconds = Math.max(0, Math.round((Date.now() - state.startedAt - activePausedMs) / 1000))
  const reps = clamp(Math.round(repsOverride ?? Math.max(1, state.completedReps || 1)), 1, 99)

  try {
    const res = await fetch('/api/progress', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'same-origin',
      body: JSON.stringify({
        type: modeCfg.progressType,
        mode: modeCfg.name,
        torqueNm: state.targetTorque,
        reps,
        durationSeconds,
        source: state.deviceMode === 'demo' ? 'demo' : 'connected-preview',
      }),
    })
    const data = await res.json()
    if (!res.ok || !data.ok) throw new Error(data.message || 'Could not save session.')
    state.saved = true
    textEl.textContent = `Saved ${modeCfg.progressType} (${modeCfg.shortName} · ${reps} reps) to Track Progress ✓`
    if (data.session) {
      state.sessions = [data.session, ...state.sessions.filter((item) => item.id !== data.session.id)]
      renderHistory()
    }
    void loadHistory()
  } catch (error) {
    feedback.classList.add('error')
    textEl.textContent = `${error.message} Click Retry Save.`
    retryBtn.hidden = false
  } finally {
    state.saving = false
  }
}

function enterActiveSession() {
  const modeCfg = activeModeConfig()
  state.active = true
  state.ended = false
  state.saving = false
  state.saved = false
  state.completedReps = 0
  state.paused = false
  state.estopped = false
  state.startedAt = Date.now()
  state.pausedAt = 0
  state.pausedMs = 0
  autoRepAccumulator = 0
  state.visualTorque = 0
  state.liveTorque = 0

  $('setupView').hidden = true
  $('activeView').hidden = false
  $('saveFeedback').hidden = true

  $('activeModePill').textContent = modeCfg.name
  $('activeSuggestedTag').textContent = `Therapist suggested: ${modeCfg.suggestedReps} reps`
  $('torqueRoleEyebrow').textContent = modeCfg.eyebrow
  $('torqueOdometerLabel').textContent = modeCfg.roleLabel
  $('setpointLabel').textContent = modeCfg.setpointLabel
  $('odoMaxLabel').textContent = String(modeCfg.maxTorque)
  $('odoProgress').classList.toggle('support-arc', modeCfg.role === 'support')
  $('pauseSessionBtn').textContent = 'Pause Session'
  $('pauseSessionBtn').disabled = false
  $('endSessionBtn').disabled = false
  $('autoSimToggleBtn').textContent = state.demoRunning ? 'Pause Auto Demo' : 'Resume Auto Demo'

  paintTorqueOdometer(0)
  updateCountdownUI()
  updateTorqueControlsUI()

  const { sentLive } = sendEspCommand({
    type: 'start_session',
    mode: modeCfg.id,
    op_mode: modeCfg.opMode,
    target_reps: state.targetReps,
    torque_nm: state.targetTorque,
  })
  $('espCommandStatus').textContent = sentLive
    ? `Session started on ESP · Target: ${state.targetTorque} N·m`
    : `Initial target ${state.targetTorque} N·m — adjust with − / + to send to ESP`

  connectEspSocket()
}

function endActiveSession() {
  if (!state.active || state.ended) return
  state.ended = true
  if (!state.paused) {
    state.paused = true
    state.pausedAt = Date.now()
  }
  stopDemoTelemetry()
  sendEspCommand({ type: 'stop_session' })
  $('pauseSessionBtn').disabled = true
  $('endSessionBtn').disabled = true
  updateCountdownUI()
  updateEspStatusPill()
  void saveSessionProgress(Math.max(1, state.completedReps || 1))
}

function exitToSetup() {
  if (state.active) {
    sendEspCommand({ type: 'stop_session' })
  }
  state.active = false
  state.ended = false
  stopDemoTelemetry()
  closeEspSocket()
  $('activeView').hidden = true
  $('setupView').hidden = false
  updateSetupUI()
}

function togglePauseSession() {
  if (state.ended) return
  if (state.paused) {
    if (state.pausedAt) state.pausedMs += Date.now() - state.pausedAt
    state.pausedAt = 0
    state.paused = false
    $('pauseSessionBtn').textContent = 'Pause Session'
    sendEspCommand({ type: 'resume_session' })
    if (!state.wsConnected && state.demoRunning) startDemoTelemetry()
  } else {
    state.paused = true
    state.pausedAt = Date.now()
    $('pauseSessionBtn').textContent = 'Resume Session'
    sendEspCommand({ type: 'pause_session' })
  }
  updateEspStatusPill()
}

function attachEvents() {
  $('menuToggleBtn').addEventListener('click', () => {
    const drawer = $('mobileNavDrawer')
    const open = drawer.classList.toggle('open')
    $('menuToggleBtn').setAttribute('aria-expanded', String(open))
  })

  document.querySelectorAll('.mode-card').forEach((card) => {
    card.addEventListener('click', () => {
      const modeId = card.dataset.mode
      if (!MODES[modeId]) return
      state.mode = modeId
      state.targetReps = MODES[modeId].suggestedReps
      state.targetTorque = MODES[modeId].defaultTorque
      updateSetupUI()
    })
  })

  $('useSuggestedBtn').addEventListener('click', () => {
    state.targetReps = activeModeConfig().suggestedReps
    updateSetupUI()
  })

  $('repMinusBtn').addEventListener('click', () => {
    state.targetReps = clamp(state.targetReps - 1, MIN_REPS, MAX_REPS)
    updateSetupUI()
  })

  $('repPlusBtn').addEventListener('click', () => {
    state.targetReps = clamp(state.targetReps + 1, MIN_REPS, MAX_REPS)
    updateSetupUI()
  })

  $('startWorkoutBtn').addEventListener('click', enterActiveSession)
  $('backToSetupBtn').addEventListener('click', exitToSetup)
  $('chooseAnotherModeBtn').addEventListener('click', exitToSetup)
  $('endSessionBtn').addEventListener('click', endActiveSession)
  $('retrySaveBtn').addEventListener('click', () => void saveSessionProgress(Math.max(1, state.completedReps || 1)))

  $('repeatSessionBtn').addEventListener('click', () => {
    state.ended = false
    state.saved = false
    state.saving = false
    state.completedReps = 0
    state.paused = false
    state.estopped = false
    state.startedAt = Date.now()
    state.pausedAt = 0
    state.pausedMs = 0
    autoRepAccumulator = 0
    $('pauseSessionBtn').textContent = 'Pause Session'
    $('pauseSessionBtn').disabled = false
    $('endSessionBtn').disabled = false
    $('saveFeedback').hidden = true
    updateCountdownUI()
    updateEspStatusPill()
    if (!state.wsConnected && state.demoRunning) startDemoTelemetry()
  })

  $('torqueMinusBtn').addEventListener('click', () => adjustTorque(-activeModeConfig().stepTorque))
  $('torquePlusBtn').addEventListener('click', () => adjustTorque(activeModeConfig().stepTorque))

  $('pauseSessionBtn').addEventListener('click', togglePauseSession)

  $('hardStopBtn').addEventListener('click', () => {
    state.estopped = true
    if (!state.paused) {
      state.paused = true
      state.pausedAt = Date.now()
    }
    $('pauseSessionBtn').textContent = 'Resume Session'
    setLiveTorque(0)
    sendEspCommand({ type: 'estop', estop_active: 1 })
    updateEspStatusPill()
    $('espCommandStatus').textContent = 'Hard Stop command sent to ESP.'
  })

  $('simRepBtn').addEventListener('click', () => {
    if (state.ended || state.completedReps >= state.targetReps) return
    state.completedReps += 1
    updateCountdownUI()
  })

  $('autoSimToggleBtn').addEventListener('click', () => {
    state.demoRunning = !state.demoRunning
    $('autoSimToggleBtn').textContent = state.demoRunning ? 'Pause Auto Demo' : 'Resume Auto Demo'
    if (state.demoRunning) startDemoTelemetry()
    else stopDemoTelemetry()
    updateEspStatusPill()
  })
}

async function loadDeviceConfig() {
  try {
    const res = await fetch('/api/device/config', { credentials: 'same-origin' })
    if (!res.ok) return
    const data = await res.json()
    if (data.ok) {
      state.bypassEsp = Boolean(data.bypassEsp)
      state.deviceMode = data.deviceMode === 'connected' ? 'connected' : 'demo'
      state.wsUrl = data.wsUrl || ''
      updateEspStatusPill()
    }
  } catch {}
}

window.doulaFlexion = { updateFromESP }

buildOdometerTicks()
attachEvents()
updateSetupUI()
void loadDeviceConfig()
void loadHistory()
