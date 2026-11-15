const root = document.querySelector('#app')
const defaultIntake = () => ({
  pathology: 'ACLR',
  weeksPostOp: 4,
  painVas: 3,
  effusion: 'trace',
  romExt: 5,
  romFlex: 105,
  bodyMassKg: 70,
  injuredLimb: 'Right',
  dOffsetCm: 9,
  lLegCm: 30,
})
const PATHOLOGY_OPTIONS = [
  ['ACLR', 'ACL Reconstruction (ACLR)'],
  ['TKA', 'Total Knee Arthroplasty (TKA)'],
  ['Meniscus', 'Meniscal Repair'],
  ['PCL', 'PCL Reconstruction'],
  ['PFPS', 'Patellofemoral Pain (PFPS)'],
  ['General', 'General Strengthening'],
]
const EFFUSION_OPTIONS = [
  ['none', 'None'],
  ['trace', 'Trace'],
  ['moderate', 'Moderate'],
  ['severe', 'Severe'],
]

const state = {
  view: 'welcome', name: '', email: '', password: '', otp: ['', '', '', ''],
  user: null, busy: false, message: '', info: false, menu: false,
  connection: '', connectionError: false, connected: false, connectionMode: 'none', espUrl: '',
  progress: [], progressError: '',
  intake: defaultIntake(),
}
const escapeHtml = value => String(value ?? '').replace(/[&<>"']/g, character => ({ '&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;', "'":'&#39;' }[character]))

function applySessionState(result) {
  state.user = result?.user ?? null
  state.connectionMode = result?.connectionMode || 'none'
  state.connected = state.connectionMode === 'connected'
  if (typeof result?.espUrl === 'string') {
    state.espUrl = result.espUrl
  }
  if (state.user?.clinicalIntake) {
    state.intake = { ...defaultIntake(), ...state.user.clinicalIntake }
  } else if (!state.user) {
    state.intake = defaultIntake()
  }
}

async function api(path, options = {}) {
  const response = await fetch('/api' + path, {
    headers: options.body ? { 'Content-Type': 'application/json' } : {},
    credentials: 'same-origin',
    ...options,
    body: options.body ? JSON.stringify(options.body) : undefined
  })
  const data = await response.json()
  if (!response.ok || !data.ok) throw new Error(data.message || 'Something went wrong.')
  return data
}
function navigate(path) {
  if (path === '/device' || path === '/device/') {
    window.location.assign('/device/')
    return
  }
  history.pushState({}, '', path)
  state.menu = false
  state.message = ''
  render()
  if (path === '/progress' && state.user) void loadProgress()
  scrollTo({ top: 0, behavior: 'instant' })
}
async function loadProgress() {
  if (!state.user) return
  try {
    const result = await api('/progress')
    state.progress = result.sessions
    state.progressError = ''
  } catch (error) { state.progressError = error.message || 'Could not load sessions.' }
  if (location.pathname === '/progress') render()
}
function alertMarkup() {
  return state.message ? `<p class="auth-alert ${state.info ? 'info' : ''}" role="status">${escapeHtml(state.message)}</p>` : ''
}
function formField(name, label, value, type, placeholder, autocomplete) {
  return `<label><span class="form-label">${label}</span><input class="form-input" id="${name}" name="${name}" type="${type}" value="${escapeHtml(value)}" placeholder="${placeholder}" autocomplete="${autocomplete}" required /></label>`
}
function passwordField(autocomplete) {
  return `<label><span class="form-label">Password</span><span class="password-wrap"><input class="form-input" id="password" name="password" type="password" autocomplete="${autocomplete}" minlength="6" placeholder="At least 6 characters" required /><button class="show-password" type="button" id="toggle-password" aria-label="Show password">◉</button></span></label>`
}
function authPage() {
  const view = state.view
  const back = view === 'welcome' ? '' : `<button class="back-button" id="auth-back" type="button" aria-label="Go back">←</button>`
  let content = ''
  if (view === 'welcome') content = `
    <div class="auth-icon" aria-hidden="true">✳</div>
    <p class="eyebrow">Welcome to DOULA</p>
    <h1 class="auth-title">Your recovery.<br />Your space.</h1>
    <p class="auth-subtitle">Sign in to access the DOULA knee rehabilitation dashboard, or create a new account with Gmail.</p>
    <div class="auth-action-area"><button class="primary-button" id="go-login">Log in <span aria-hidden="true">→</span></button><button class="secondary-button" id="go-signup">Make a new account</button></div>`
  if (view === 'login') content = `
    <form id="login-form" class="auth-form"><p class="eyebrow">Welcome back</p><h1 class="auth-title">Log in</h1><p class="auth-subtitle">Enter your Gmail address and password.</p>
      <div class="form-fields">${formField('email','Google ID',state.email,'email','name@gmail.com','email')}${passwordField('current-password')}</div>${alertMarkup()}
      <div class="auth-form-footer"><button class="primary-button" type="submit" ${state.busy?'disabled':''}>${state.busy?'Logging in…':'Log in →'}</button><div class="auth-switch">New to DOULA? <button type="button" id="go-signup">Create account</button></div></div></form>`
  if (view === 'signup') content = `
    <form id="signup-form" class="auth-form"><p class="eyebrow">New account</p><h1 class="auth-title">Get started.</h1><p class="auth-subtitle">Enter your name and Gmail address to receive a four-digit OTP.</p>
      <div class="form-fields">${formField('name','Your name',state.name,'text','Your name','name')}${formField('email','Google ID',state.email,'email','name@gmail.com','email')}</div>${alertMarkup()}
      <div class="auth-form-footer"><button class="primary-button" type="submit" ${state.busy?'disabled':''}>${state.busy?'Sending OTP…':'Get OTP →'}</button><div class="auth-switch">Already have an account? <button type="button" id="go-login">Log in</button></div></div></form>`
  if (view === 'verify') content = `
    <form id="verify-form" class="auth-form"><div class="auth-icon" aria-hidden="true">✉</div><p class="eyebrow">Email verification</p><h1 class="auth-title">Enter your code.</h1><p class="auth-subtitle">Enter the four-digit OTP for <strong>${escapeHtml(state.email)}</strong>.</p>
      <div class="otp-row">${state.otp.map((digit,i)=>`<input class="otp-digit" data-digit="${i}" aria-label="OTP digit ${i+1}" inputmode="numeric" pattern="[0-9]*" maxlength="1" value="${digit}" autocomplete="off" />`).join('')}</div>
      <p class="note">If Gmail email delivery isn't configured yet, find the OTP in the terminal running DOULA.</p>${alertMarkup()}
      <div class="auth-form-footer"><button class="primary-button" type="submit" ${state.busy?'disabled':''}>${state.busy?'Checking…':'Verify OTP →'}</button></div></form>`
  if (view === 'password') content = `
    <form id="password-form" class="auth-form"><div class="auth-icon" aria-hidden="true">✓</div><p class="eyebrow">Gmail verified</p><h1 class="auth-title">Set a password.</h1><p class="auth-subtitle">One password, no extra steps. You'll use it to log in next time.</p>
      <div class="form-fields">${passwordField('new-password')}</div>${alertMarkup()}
      <div class="auth-form-footer"><button class="primary-button" type="submit" ${state.busy?'disabled':''}>${state.busy?'Creating…':'Create account →'}</button></div></form>`
  return `<main class="auth-page"><section class="auth-card"><header class="auth-head"><div class="auth-brand"><span class="brand-mark">D</span><span>DOULA</span></div>${back}</header><div class="auth-body">${content}</div></section></main>`
}
function header() {
  const p = location.pathname
  return `<header class="site-header"><a class="site-logo" href="/dashboard" data-link>DOULA</a>
    <nav class="desktop-nav" aria-label="Main navigation"><ul class="nav-list">
      <li><a href="/intake" data-link class="${p==='/intake'?'active':''}">Clinical Intake</a></li>
      <li><a href="/progress" data-link class="${p==='/progress'?'active':''}">Track Progress</a></li>
      <li><a href="/how-to" data-link class="${p==='/how-to'?'active':''}">How-to</a></li>
      <li><a href="/about" data-link class="${p==='/about'?'active':''}">About Us</a></li>
    </ul></nav>
    <button type="button" class="logout-btn top-logout" data-logout>Log Out ↗</button>
    <button class="hamburger" type="button" id="toggle-menu" aria-label="Toggle menu" aria-expanded="${state.menu}">${state.menu?'×':'☰'}</button>
    ${state.menu ? `<nav aria-label="Mobile navigation"><ul class="nav-list mobile-nav"><li><a data-link href="/dashboard">Home</a></li><li><a data-link href="/intake">Clinical Intake</a></li><li><a data-link href="/progress">Track Progress</a></li><li><a data-link href="/how-to">How-to</a></li><li><a data-link href="/about">About Us</a></li><li><button class="menu-logout" type="button" data-logout>Log Out ↗</button></li></ul></nav>`:''}
  </header>`
}
function dashboardClinicalSummary() {
  const rx = state.user?.prescription
  if (!rx) {
    return `<section class="dashboard-clinical-box" aria-label="Clinical intake reminder">
      <div class="dashboard-clinical-top">
        <div>
          <div class="micro-label">STEP 1 · CLINICAL DECISION TREE (U1)</div>
          <h3>Personalize your training regime</h3>
        </div>
        <a class="demo-btn" href="/intake" data-link>Take Questionnaire →</a>
      </div>
      <p>Complete the clinical questionnaire (post-op weeks, VAS pain, effusion, and safe ROM) so DOULA can suggest your safe sessions and settings before you exercise.</p>
    </section>`
  }
  const flex = rx.flexion
  const cyc = rx.cycling
  const primaryText = rx.recommendedSession === 'flexion'
    ? 'Start with Flexion & Extension'
    : rx.recommendedSession === 'cycling'
      ? 'Prioritize Cycling + Curve Isotonic'
      : 'Cleared for Combined Cycling + Flexion'
  return `<section class="dashboard-clinical-box" aria-label="Recommended training regime">
    <div class="dashboard-clinical-top">
      <div>
        <div class="micro-label">${escapeHtml(rx.phase)}</div>
        <h3>${escapeHtml(rx.headline)}</h3>
      </div>
      <a class="inline-link" style="margin-top:0;" href="/intake" data-link>Update Assessment</a>
    </div>
    <p>${escapeHtml(rx.summary)}</p>
    <div class="dashboard-clinical-pills">
      <span class="dashboard-clinical-pill">${escapeHtml(primaryText)}</span>
      <span class="dashboard-clinical-pill">Flexion: ${escapeHtml(flex.modeName)} · ${Number(flex.targetTorqueNm)} N·m · ${Number(flex.suggestedReps)} reps</span>
      <span class="dashboard-clinical-pill ${cyc.allowed ? '' : 'warn'}">${cyc.allowed ? `Cycling: ${escapeHtml(cyc.modeName)} · ${Number(cyc.targetTorqueNm)} N·m (${escapeHtml(cyc.cadenceRangeRpm)})` : `Cycling: ${escapeHtml(cyc.priorityLabel)}`}</span>
    </div>
  </section>`
}
function dashboard() {
  return `${header()}<main class="dashboard-main"><h1 class="hero-title">Knee Rehabilitation</h1>
    <div class="hero-art"><div class="sage-band"></div><div class="photo-frame"><img src="/assets/rehabilitation.jpg" alt="Person receiving support during knee rehabilitation" /></div></div>
    <section class="hero-bottom">${dashboardClinicalSummary()}<div class="micro-label">DOULA: Knee Rehabilitation</div><h2>Start exercising</h2><p>Check if the rehabilitation device is connected</p>
      <div class="esp-url-row"><input class="esp-url-input" id="esp-url-input" type="text" placeholder="ESP32 IP or WS URL (e.g. 192.168.1.42 or ws://192.168.1.42:81/)" value="${escapeHtml(state.espUrl)}" aria-label="ESP32 IP address or WebSocket URL" /></div>
      <div class="dashboard-connection-actions"><button class="connect-btn" id="connect-btn" type="button" ${state.busy?'disabled':''}>${state.busy?'Checking…':'Connect Device'}</button><button class="demo-btn" id="demo-btn" type="button">Try without ESP32 ↗</button></div>
      <p class="demo-caption">Leave the address blank to use <code>ESP32_WS_URL</code> in <code>.env</code>, or click Try without ESP32 for demo mode.</p>
      ${state.connection?`<p class="connect-status ${state.connectionError?'error':''}" role="status">${escapeHtml(state.connection)}</p>`:''}
    </section></main>`
}
function prescriptionResultMarkup(rx) {
  if (!rx) return ''
  const flex = rx.flexion
  const cyc = rx.cycling
  const isFlexPrimary = rx.recommendedSession === 'flexion' || rx.recommendedSession === 'both'
  const isCycPrimary = cyc.allowed && (rx.recommendedSession === 'cycling' || rx.recommendedSession === 'both')
  return `<section class="clinical-result-card" id="clinical-result-card" aria-label="Recommended training regime">
    <span class="clinical-phase-pill">${escapeHtml(rx.phase)}</span>
    <h2>${escapeHtml(rx.headline)}</h2>
    <p class="clinical-summary-text">${escapeHtml(rx.summary)}</p>
    <div class="clinical-locks">${rx.safetyLocks.map(lock => `<div class="clinical-lock-pill ${/Locked|Contraindicated|Precaution/i.test(lock) ? 'caution' : ''}">${escapeHtml(lock)}</div>`).join('')}</div>
    <div class="clinical-regime-grid">
      <article class="regime-box ${isFlexPrimary ? 'recommended' : ''}">
        <div class="regime-box-head"><strong>Flexion &amp; Extension Settings</strong><span class="regime-badge">${isFlexPrimary ? 'Recommended' : 'Option'}</span></div>
        <ul class="regime-list">
          <li><strong>Mode:</strong> ${escapeHtml(flex.clinicalLabel)}</li>
          <li><strong>Target Torque:</strong> ${Number(flex.targetTorqueNm)} N·m (${escapeHtml(flex.roleLabel)})</li>
          <li><strong>Repetitions:</strong> ${Number(flex.suggestedReps)} reps</li>
          <li><strong>ROM &amp; Speed:</strong> ${Number(flex.romWindowDeg[0])}°–${Number(flex.romWindowDeg[1])}° · ${escapeHtml(flex.speedSpec)}</li>
        </ul>
      </article>
      <article class="regime-box ${isCycPrimary ? 'recommended' : ''}">
        <div class="regime-box-head"><strong>Bottom-Tier Cycling Settings</strong><span class="regime-badge ${cyc.allowed ? '' : 'locked'}">${cyc.allowed ? (rx.recommendedSession === 'cycling' ? 'Primary Choice' : 'Cleared') : 'Locked (< 100° / Acute)'}</span></div>
        <ul class="regime-list">
          <li><strong>Mode:</strong> ${escapeHtml(cyc.modeName)}</li>
          <li><strong>Pedal Torque:</strong> ${escapeHtml(cyc.torqueRangeNm)}</li>
          <li><strong>Cadence &amp; Power:</strong> ${escapeHtml(cyc.cadenceRangeRpm)} · ${escapeHtml(cyc.powerRangeW)}</li>
          <li><strong>Note:</strong> ${escapeHtml(cyc.note)}</li>
        </ul>
      </article>
    </div>
    <div class="gate-trace"><strong>Automated Decision Tree Evaluation</strong>${rx.decisionPath.map(step => `<div>• ${escapeHtml(step)}</div>`).join('')}</div>
    <div class="clinical-actions">
      <button class="connect-btn" type="button" id="intake-demo-btn">Choose Session in Demo Mode →</button>
      <a class="demo-btn" href="/dashboard" data-link>Go to Dashboard</a>
    </div>
  </section>`
}
function intakePage() {
  const f = state.intake
  const rx = state.user?.prescription
  const flexOk = Number(f.romFlex) >= 100
  const vasClass = Number(f.painVas) >= 6 ? 'alert' : Number(f.painVas) >= 4 ? 'warn' : ''
  return `${header()}<main class="inner-page intake-page">
    <div class="inner-eyebrow">DOULA · Automated Clinical Decision Tree (Section 3.2)</div>
    <h1 class="inner-title">Clinical Intake</h1>
    <p class="inner-description">Complete these post-operative criteria for ${escapeHtml(state.user?.name || 'your profile')} so DOULA can prescribe your training regime and pre-fill safe settings before every session.</p>
    <form id="intake-form" class="intake-form">
      <section class="intake-card">
        <div class="intake-step-eyebrow">1. Pathology / Surgical Classification</div>
        <h2>What condition are you rehabilitating?</h2>
        <div class="choice-grid" role="radiogroup" aria-label="Pathology">
          ${PATHOLOGY_OPTIONS.map(([id, label]) => `<button type="button" class="choice-chip-btn ${f.pathology === id ? 'selected' : ''}" data-pathology="${id}" aria-checked="${f.pathology === id}">${escapeHtml(label)}</button>`).join('')}
        </div>
      </section>

      <section class="intake-card">
        <div class="intake-step-eyebrow">2. Timeline, Pain (VAS) &amp; Swelling</div>
        <h2>Current clinical status</h2>
        <div class="intake-two-col">
          <div class="intake-field">
            <label for="intake-weeks">Weeks since surgery / injury</label>
            <input class="intake-number" id="intake-weeks" type="number" min="0" max="260" step="0.5" value="${Number(f.weeksPostOp)}" required />
            <div class="preset-pill-row">
              ${[[0.5, '< 1 wk'], [2, '2 wks'], [4, '4 wks'], [6, '6 wks'], [10, '10+ wks']].map(([val, lbl]) => `<button type="button" class="preset-mini-btn ${Number(f.weeksPostOp) === val ? 'selected' : ''}" data-week-preset="${val}">${lbl}</button>`).join('')}
            </div>
          </div>
          <div class="intake-field">
            <div class="intake-slider-head">
              <label for="intake-vas" style="margin:0;">Visual Analog Pain Scale (VAS 0–10)</label>
              <span class="intake-slider-val ${vasClass}" id="intake-vas-label">${Number(f.painVas)} / 10</span>
            </div>
            <input class="intake-range" id="intake-vas" type="range" min="0" max="10" step="1" value="${Number(f.painVas)}" />
            <div class="preset-pill-row" style="justify-content:space-between;font-size:11.5px;color:#6d7c70;">
              <span>0 · Pain-free</span><span>3–5 · Mild/Mod</span><span>6+ · Acute</span>
            </div>
          </div>
        </div>
        <div class="intake-field" style="margin-top:18px;">
          <label>Joint swelling / effusion grade</label>
          <div class="choice-grid choice-grid-4" role="radiogroup" aria-label="Effusion grade">
            ${EFFUSION_OPTIONS.map(([id, label]) => `<button type="button" class="choice-chip-btn ${f.effusion === id ? 'selected' : ''}" data-effusion="${id}" aria-checked="${f.effusion === id}">${escapeHtml(label)}</button>`).join('')}
          </div>
        </div>
      </section>

      <section class="intake-card">
        <div class="intake-step-eyebrow">3. Measured Pain-Free Range of Motion (ROM)</div>
        <h2>Safe flexion &amp; extension envelope</h2>
        <div class="intake-two-col">
          <div class="intake-field">
            <div class="intake-slider-head">
              <label for="intake-flex" style="margin:0;">Max Pain-Free Flexion (θ_flex)</label>
              <span class="intake-slider-val ${flexOk ? '' : 'warn'}" id="intake-flex-label">${Number(f.romFlex)}° · ${flexOk ? 'Cycling Unlocked' : 'Cycling Locked'}</span>
            </div>
            <input class="intake-range" id="intake-flex" type="range" min="15" max="145" step="1" value="${Number(f.romFlex)}" />
          </div>
          <div class="intake-field">
            <div class="intake-slider-head">
              <label for="intake-ext" style="margin:0;">Extension Limit / Lag (θ_ext)</label>
              <span class="intake-slider-val" id="intake-ext-label">${Number(f.romExt)}°</span>
            </div>
            <input class="intake-range" id="intake-ext" type="range" min="0" max="45" step="1" value="${Number(f.romExt)}" />
          </div>
        </div>
      </section>

      <section class="intake-card">
        <div class="intake-step-eyebrow">4. Offset-Pivot Geometry &amp; Profile</div>
        <h2>Device fitment parameters</h2>
        <div class="intake-four-col">
          <div class="intake-field">
            <label for="intake-mass">Body Mass (kg)</label>
            <input class="intake-number" id="intake-mass" type="number" min="25" max="250" step="0.5" value="${Number(f.bodyMassKg)}" required />
          </div>
          <div class="intake-field">
            <label for="intake-limb">Injured Limb</label>
            <select class="intake-number" id="intake-limb">
              <option value="Right" ${f.injuredLimb === 'Right' ? 'selected' : ''}>Right</option>
              <option value="Left" ${f.injuredLimb === 'Left' ? 'selected' : ''}>Left</option>
            </select>
          </div>
          <div class="intake-field">
            <label for="intake-offset">Offset d_offset (cm)</label>
            <input class="intake-number" id="intake-offset" type="number" min="5" max="15" step="0.5" value="${Number(f.dOffsetCm)}" required />
          </div>
          <div class="intake-field">
            <label for="intake-leg">Pad Dist L_leg (cm)</label>
            <input class="intake-number" id="intake-leg" type="number" min="15" max="50" step="0.5" value="${Number(f.lLegCm)}" required />
          </div>
        </div>
      </section>

      ${alertMarkup()}
      <div class="intake-submit-row">
        <button class="intake-submit-btn" type="submit" ${state.busy ? 'disabled' : ''}>${state.busy ? 'Computing Regime…' : 'Generate Recommended Regime →'}</button>
        ${rx ? '<a class="inline-link" style="margin-top:0;" href="/dashboard" data-link>← Back to dashboard</a>' : ''}
      </div>
    </form>
    ${prescriptionResultMarkup(rx)}
  </main>`
}
function progressPage() {
  const sessions = state.progress
  const totalReps = sessions.reduce((sum, row) => sum + Number(row.reps || 0), 0)
  const modes = new Set(sessions.map(row => row.type)).size
  const rows = sessions.map(row => {
    const day = new Date(row.date)
    const date = Number.isNaN(day.getTime()) ? escapeHtml(row.date || '') : escapeHtml(day.toLocaleString('en-IN', { day: 'numeric', month: 'short', year: 'numeric', hour: 'numeric', minute: '2-digit' }))
    const modeTag = row.source === 'esp32' ? (row.verifiedByDevice ? 'ESP32 Verified' : 'ESP32 Live') : row.source === 'demo' ? 'Demo' : 'Connected preview'
    return `<tr><td>${date}</td><td><strong>${escapeHtml(row.type)}</strong>${row.mode && row.mode !== row.type ? `<div class="progress-subtype">${escapeHtml(row.mode)}${Number.isFinite(Number(row.torqueNm)) ? ` · ${Number(row.torqueNm)} N·m` : ''}</div>` : ''}</td><td>${Number(row.reps)}</td><td><span class="trial-chip">${modeTag}</span></td></tr>`
  }).join('')
  return `${header()}<main class="inner-page progress-page"><div class="inner-eyebrow">DOULA: Rehabilitation records</div><h1 class="inner-title">Track Progress</h1><p class="inner-description">Your completed exercise sessions, saved to your account on this computer.</p>
    <section class="progress-summary"><article><span>Sessions completed</span><strong>${sessions.length}</strong></article><article><span>Simulated repetitions</span><strong>${totalReps}</strong></article><article><span>Exercise types tried</span><strong>${modes}</strong></article></section>
    <section class="surface-panel"><h2>Previous exercises</h2>${state.progressError ? `<p class="progress-error" role="alert">${escapeHtml(state.progressError)}</p>` : ''}${sessions.length ? `<div class="progress-table-wrap"><table class="simple-table"><thead><tr><th>Date & time</th><th>Exercise</th><th>Reps</th><th>Mode</th></tr></thead><tbody>${rows}</tbody></table></div>` : '<p>No sessions recorded yet. Complete a Cycling or Flexion &amp; Extension session to create your first entry.</p>'}</section><a class="inline-link" href="/dashboard" data-link>← Back to home</a></main>`
}
function extraPage() {
  const path = location.pathname
  if (path === '/intake') return intakePage()
  if (path === '/progress') return progressPage()
  if (path === '/how-to') return `${header()}<main class="inner-page"><div class="inner-eyebrow">DOULA: Guide</div><h1 class="inner-title">How-to</h1><p class="inner-description">A place for your DOULA device instructions and documents.</p><section class="surface-panel"><h2>Device guide</h2><p>The operating instructions will be added when the device workflow is finalized. Please use the device only with appropriate supervision.</p></section><a href="/dashboard" class="inline-link" data-link>← Back to home</a></main>`
  if (path === '/about') return `${header()}<main class="inner-page"><div class="inner-eyebrow">Meet DOULA</div><h1 class="inner-title">About Us</h1><p class="inner-description">DOULA is a prototype knee rehabilitation device with ESP32-based controllers.</p><section class="surface-panel"><h2>Our project</h2><p>More information about the DOULA team and the device will be added here.</p></section><a href="/dashboard" class="inline-link" data-link>← Back to home</a></main>`
  if (path === '/device') return `${header()}<main class="inner-page"><h1 class="inner-title">Exercise interface</h1><p class="inner-description">Go back to the dashboard to check the ESP32 or open Demo mode.</p><a data-link href="/dashboard" class="inline-link">← Back to home</a></main>`
  return dashboard()
}
function render() {
  if (!state.user) {
    root.innerHTML = authPage()
  } else if (location.pathname === '/intake') {
    root.innerHTML = intakePage()
  } else if (location.pathname === '/dashboard' || location.pathname === '/') {
    root.innerHTML = state.user.needsIntake ? intakePage() : dashboard()
  } else {
    root.innerHTML = extraPage()
  }
  attachEvents()
}
function switchAuth(view) { state.view=view; state.message=''; state.info=false; render() }
function updateValue(name,value) { state[name]=value }
async function withPending(task) {
  if (state.busy) return
  state.busy=true;state.message='';state.info=false;render()
  try { await task() }
  catch(error) { state.message=error.message || 'Something went wrong.'; state.info=false }
  finally { state.busy=false;render() }
}
function attachEvents() {
  document.querySelector('#go-login')?.addEventListener('click',()=>switchAuth('login'))
  document.querySelector('#go-signup')?.addEventListener('click',()=>switchAuth('signup'))
  document.querySelector('#auth-back')?.addEventListener('click',()=>switchAuth(state.view==='password'?'verify':state.view==='verify'?'signup':'welcome'))
  document.querySelector('#name')?.addEventListener('input',e=>updateValue('name',e.target.value))
  document.querySelector('#email')?.addEventListener('input',e=>updateValue('email',e.target.value))
  document.querySelector('#password')?.addEventListener('input',e=>updateValue('password',e.target.value))
  document.querySelector('#toggle-password')?.addEventListener('click',()=>{
    const input=document.querySelector('#password');input.type=input.type==='password'?'text':'password'
  })
  document.querySelector('#login-form')?.addEventListener('submit',e=>{
    e.preventDefault();withPending(async()=>{
      const result=await api('/auth/login',{method:'POST',body:{email:state.email,password:state.password}})
      applySessionState(result);state.password='';navigate('/dashboard')
    })
  })
  document.querySelector('#signup-form')?.addEventListener('submit',e=>{
    e.preventDefault();withPending(async()=>{
      const result=await api('/auth/start',{method:'POST',body:{name:state.name,email:state.email}})
      state.otp=['','','',''];state.view='verify';state.message=result.message;state.info=true
    })
  })
  const digits=[...document.querySelectorAll('.otp-digit')]
  digits.forEach((input,i)=>{
    input.addEventListener('input',e=>{
      state.otp[i]=input.value.replace(/\D/g,'').slice(-1);input.value=state.otp[i]
      if(state.otp[i]&&i<3)digits[i+1].focus()
    })
    input.addEventListener('keydown',e=>{if(e.key==='Backspace'&&!input.value&&i>0)digits[i-1].focus()})
    input.addEventListener('paste',e=>{
      const text=e.clipboardData.getData('text').replace(/\D/g,'').slice(0,4)
      if(text.length>1){e.preventDefault();state.otp=text.padEnd(4,'').split('');digits.forEach((d,j)=>d.value=state.otp[j]);digits[Math.min(text.length,3)].focus()}
    })
  })
  document.querySelector('#verify-form')?.addEventListener('submit',e=>{
    e.preventDefault();withPending(async()=>{
      await api('/auth/verify',{method:'POST',body:{email:state.email,code:state.otp.join('')}})
      state.view='password';state.password=''
    })
  })
  document.querySelector('#password-form')?.addEventListener('submit',e=>{
    e.preventDefault();withPending(async()=>{
      const result=await api('/auth/finish',{method:'POST',body:{password:state.password}})
      applySessionState(result);state.password='';navigate(result.user?.needsIntake ? '/intake' : '/dashboard')
    })
  })
  document.querySelectorAll('[data-pathology]').forEach(btn=>btn.addEventListener('click',()=>{
    state.intake.pathology=btn.dataset.pathology;render()
  }))
  document.querySelectorAll('[data-effusion]').forEach(btn=>btn.addEventListener('click',()=>{
    state.intake.effusion=btn.dataset.effusion;render()
  }))
  document.querySelectorAll('[data-week-preset]').forEach(btn=>btn.addEventListener('click',()=>{
    state.intake.weeksPostOp=Number(btn.dataset.weekPreset);render()
  }))
  document.querySelector('#intake-weeks')?.addEventListener('input',e=>{ state.intake.weeksPostOp=Number(e.target.value) })
  document.querySelector('#intake-vas')?.addEventListener('input',e=>{
    const val=Number(e.target.value)
    state.intake.painVas=val
    const lbl=document.querySelector('#intake-vas-label')
    if(lbl){ lbl.textContent=`${val} / 10`; lbl.className=`intake-slider-val ${val>=6?'alert':val>=4?'warn':''}` }
  })
  document.querySelector('#intake-flex')?.addEventListener('input',e=>{
    const val=Number(e.target.value)
    state.intake.romFlex=val
    const lbl=document.querySelector('#intake-flex-label')
    if(lbl){ lbl.textContent=`${val}° · ${val>=100?'Cycling Unlocked':'Cycling Locked'}`; lbl.className=`intake-slider-val ${val>=100?'':'warn'}` }
  })
  document.querySelector('#intake-ext')?.addEventListener('input',e=>{
    const val=Number(e.target.value)
    state.intake.romExt=val
    const lbl=document.querySelector('#intake-ext-label')
    if(lbl) lbl.textContent=`${val}°`
  })
  document.querySelector('#intake-mass')?.addEventListener('input',e=>{ state.intake.bodyMassKg=Number(e.target.value) })
  document.querySelector('#intake-limb')?.addEventListener('change',e=>{ state.intake.injuredLimb=e.target.value })
  document.querySelector('#intake-offset')?.addEventListener('input',e=>{ state.intake.dOffsetCm=Number(e.target.value) })
  document.querySelector('#intake-leg')?.addEventListener('input',e=>{ state.intake.lLegCm=Number(e.target.value) })
  document.querySelector('#intake-form')?.addEventListener('submit',e=>{
    e.preventDefault();withPending(async()=>{
      const result=await api('/clinical/intake',{method:'POST',body:state.intake})
      applySessionState(result)
      history.replaceState({}, '', '/intake')
      state.info=true
      state.message='Clinical intake saved. Your recommended training regime is ready below.'
    })
  })
  document.querySelector('#intake-demo-btn')?.addEventListener('click',()=>withPending(async()=>{
    await api('/device/demo',{method:'POST'})
    state.connected=false;state.connectionMode='demo';state.connection='';state.connectionError=false
    window.location.assign('/device/')
  }))
  document.querySelector('#toggle-menu')?.addEventListener('click',()=>{state.menu=!state.menu;render()})
  document.querySelectorAll('[data-link]').forEach(el=>el.addEventListener('click',e=>{e.preventDefault();navigate(el.getAttribute('href'))}))
  document.querySelectorAll('[data-logout]').forEach(el=>el.addEventListener('click',()=>withPending(async()=>{
    await api('/auth/logout',{method:'POST'});state.user=null;state.view='login';state.email='';state.password='';state.menu=false;state.connected=false;state.connectionMode='none';state.connection='';state.progress=[];navigate('/')
  })))
  document.querySelector('#demo-btn')?.addEventListener('click',()=>withPending(async()=>{
    await api('/device/demo',{method:'POST'})
    state.connected=false;state.connectionMode='demo';state.connection='';state.connectionError=false
    window.location.assign('/device/')
  }))
  document.querySelector('#esp-url-input')?.addEventListener('input', e => updateValue('espUrl', e.target.value))
  document.querySelector('#connect-btn')?.addEventListener('click',()=>withPending(async()=>{
    const trimmed = String(state.espUrl || '').trim()
    const result = trimmed
      ? await api('/device/connect', { method: 'POST', body: { wsUrl: trimmed } })
      : await api('/device/status')
    if (result.wsUrl) state.espUrl = result.wsUrl
    if(result.connected){
      state.connected=true;state.connectionMode='connected';state.connection='';state.connectionError=false
      window.location.assign('/device/')
    } else { state.connection=result.message;state.connectionError=true }
  }))
}
window.addEventListener('popstate',()=>{render();if(location.pathname==='/progress'&&state.user)void loadProgress()})
;(async()=>{
  try {
    const result=await api('/me')
    applySessionState(result)
    if(state.user&&location.pathname==='/progress'){
      const progress=await api('/progress');state.progress=progress.sessions
    }
    if(!state.user&&location.pathname!=='/') history.replaceState({},'','/')
  } catch { state.user=null }
  render()
})()
