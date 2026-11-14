// Figma exercise-mode design adapted to the existing DOULA browser/frontend.
// All repetitions in this version are a timed UI simulation, not ESP32 telemetry.

const boltIcon = `<svg aria-hidden="true" viewBox="0 0 32 32" fill="none" stroke="currentColor"><path d="M18.5 2.5 8 17h7l-1.5 12.5L24 14.5h-7l1.5-12Z" stroke-width="1.5" stroke-linejoin="round"/></svg>`
const cycleIcon = `<svg aria-hidden="true" viewBox="0 0 32 32" fill="none" stroke="currentColor"><circle cx="9" cy="22" r="5.5" stroke-width="1.5"/><circle cx="24" cy="22" r="5.5" stroke-width="1.5"/><path d="m9 22 5-10 4 10m-9 0h9l-5-7h8M13 9h4" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/></svg>`
const arrowIcon = `<svg aria-hidden="true" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M5 12h14m-5-5 5 5-5 5" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/></svg>`
const backIcon = `<svg aria-hidden="true" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M19 12H5m5 5-5-5 5-5" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/></svg>`

export function createExerciseUI({ state, api, render, navigate, escapeHtml, loadProgress }) {
  let timer = null
  let session = freshSession()

  function freshSession() {
    return { mode: null, flexionType: null, dialog: null, repInput: '10', activeType: null, total: 0, remaining: 0, paused: false, completed: false, saved: false, saving: false, notice: '', error: '', startedAt: 0, pausedAt: 0, pausedMs: 0 }
  }

  function stopTimer() {
    if (timer !== null) clearInterval(timer)
    timer = null
    pauseVideo()
  }

  function reset() {
    stopTimer()
    session = freshSession()
  }

  function pauseVideo() {
    const frame = document.querySelector('#exercise-video')
    if (frame?.contentWindow) frame.contentWindow.postMessage(JSON.stringify({ event: 'command', func: 'pauseVideo', args: [] }), 'https://www.youtube-nocookie.com')
  }

  function playVideo() {
    const frame = document.querySelector('#exercise-video')
    if (frame?.contentWindow) frame.contentWindow.postMessage(JSON.stringify({ event: 'command', func: 'playVideo', args: [] }), 'https://www.youtube-nocookie.com')
  }

  function selectMode(mode) {
    reset()
    session.mode = mode
    session.dialog = mode === 'cycling' ? 'Cycling' : null
    render()
  }

  function selectFlexion(type) {
    stopTimer()
    session = { ...freshSession(), mode: 'flexion', flexionType: type, dialog: type }
    render()
  }

  function begin(event) {
    event.preventDefault()
    const input = document.querySelector('#exercise-reps')
    const reps = Number(input?.value)
    if (!Number.isInteger(reps) || reps < 1 || reps > 99) {
      session.error = 'Enter a whole number of repetitions between 1 and 99.'
      render()
      return
    }
    if (!session.dialog) return
    stopTimer()
    session.activeType = session.dialog
    session.total = reps
    session.remaining = reps
    session.dialog = null
    session.repInput = String(reps)
    session.paused = false
    session.completed = false
    session.saved = false
    session.saving = false
    session.error = ''
    session.notice = ''
    session.startedAt = Date.now()
    session.pausedMs = 0
    render()
    startTimer()
  }

  function startTimer() {
    if (timer !== null) clearInterval(timer)
    timer = setInterval(() => {
      if (!session.activeType || session.paused || session.remaining < 1) return
      session.remaining -= 1
      if (session.remaining === 0) {
        if (timer !== null) clearInterval(timer)
        timer = null
        pauseVideo()
        session.completed = true
        render()
        void saveCompleted()
      } else {
        // Update the counter without rebuilding the video iframe on every tick.
        const counter = document.querySelector('#exercise-count')
        const bar = document.querySelector('#exercise-bar')
        if (counter) counter.textContent = String(session.remaining)
        if (bar) bar.style.width = `${Math.round((session.total - session.remaining) / session.total * 100)}%`
      }
    }, 3500)
    if (session.activeType === 'Cycling') playVideo()
  }

  function togglePause() {
    if (!session.activeType || session.remaining === 0) {
      session.dialog = session.mode === 'cycling' ? 'Cycling' : session.flexionType
      session.error = ''
      render()
      return
    }
    if (session.paused) {
      session.pausedMs += Date.now() - session.pausedAt
      session.paused = false
      render()
      startTimer()
    } else {
      session.paused = true
      session.pausedAt = Date.now()
      stopTimer()
      render()
    }
  }

  async function saveCompleted() {
    if (!session.completed || session.saved || session.saving) return
    session.saving = true
    session.error = ''
    render()
    try {
      const durationSeconds = Math.max(0, (Date.now() - session.startedAt - session.pausedMs) / 1000)
      const result = await api('/progress', {
        method: 'POST',
        body: {
          type: session.activeType,
          reps: session.total,
          durationSeconds,
          source: state.connectionMode === 'demo' ? 'demo' : 'connected-preview'
        }
      })
      session.saved = true
      session.notice = 'Session saved to Track Progress.'
      state.progress = [result.session, ...state.progress.filter(item => item.id !== result.session.id)]
      void loadProgress()
    } catch (error) {
      session.error = `${error.message} Your session was not saved. Click Retry save.`
    } finally {
      session.saving = false
      if (location.pathname === '/device') render()
    }
  }

  function formatDate(value) {
    if (!value) return ''
    const date = new Date(value)
    return Number.isNaN(date.getTime()) ? escapeHtml(value) : escapeHtml(date.toLocaleString('en-IN', { day: 'numeric', month: 'short', year: 'numeric', hour: 'numeric', minute: '2-digit' }))
  }

  function historyRows() {
    const visible = state.progress.filter(row => session.mode === 'cycling' ? row.type === 'Cycling' : row.type === session.flexionType)
    if (!visible.length) return `<div class="exercise-history-empty">No sessions for this exercise yet.</div>`
    return visible.slice(0, 5).map(row => `<article class="exercise-history-row">
      <div class="exercise-history-icon">${row.type === 'Cycling' ? cycleIcon : boltIcon}</div>
      <div class="exercise-history-info"><strong>${escapeHtml(row.type)}</strong><small>${formatDate(row.date)}</small></div>
      <div class="exercise-history-count"><strong>${Number(row.reps)}</strong><small>simulated reps</small></div>
    </article>`).join('')
  }

  function html() {
    const running = Boolean(session.activeType && session.remaining > 0 && !session.paused)
    const active = session.activeType !== null
    const selected = session.mode === 'cycling' ? 'Cycling' : session.flexionType || 'Flexion–Extension'
    const demo = state.connectionMode === 'demo'

    return `<div class="exercise-app">
      <aside class="exercise-sidebar">
        <div>
          <div class="exercise-brand-row"><a class="exercise-back" href="/dashboard" data-link aria-label="Back to dashboard">${backIcon}</a><strong>DOULA</strong></div>
          <h1>Select Exercise Mode</h1><p class="exercise-sidebar-intro">Click to start rehabilitation</p>
          <div class="exercise-mode-grid">
            <button type="button" data-mode="flexion" class="exercise-mode-btn ${session.mode === 'flexion' ? 'selected' : ''}" aria-pressed="${session.mode === 'flexion'}">${boltIcon}<span>Flexion–Extension</span></button>
            <button type="button" data-mode="cycling" class="exercise-mode-btn ${session.mode === 'cycling' ? 'selected' : ''}" aria-pressed="${session.mode === 'cycling'}">${cycleIcon}<span>Cycling</span></button>
          </div>
          ${session.mode === 'flexion' ? `<div class="exercise-flexion"><p>Choose assistance</p><div class="exercise-mode-grid"><button type="button" data-flexion="Support" class="exercise-submode ${session.flexionType === 'Support' ? 'selected' : ''}">Support ${arrowIcon}</button><button type="button" data-flexion="Resistance" class="exercise-submode ${session.flexionType === 'Resistance' ? 'selected' : ''}">Resistance ${arrowIcon}</button></div></div>` : ''}
          ${active ? `<button type="button" id="exercise-toggle" class="exercise-session-control"><span>${running ? 'Ⅱ' : '▶'}</span>${session.remaining === 0 ? 'Start new session' : running ? 'Pause reps' : 'Resume reps'}</button>` : ''}
        </div>
        <div class="exercise-sidebar-footer"><p>Guided rehabilitation · Session tracking</p><a data-link href="/progress">Track Progress ↗</a><a data-link href="/dashboard">Dashboard ↗</a><button type="button" data-logout>Log Out ↗</button></div>
      </aside>
      <main class="exercise-panel">
        <div class="exercise-topline"><span class="exercise-mode-indicator ${demo ? 'demo' : 'connected'}">${demo ? '● DEMO — NO ESP32' : '● ESP32 RESPONDED'}</span><a data-link href="/dashboard">← Back to dashboard</a></div>
        <p class="exercise-timer-disclaimer">${demo ? 'Trial mode: the repetition counter is simulated.' : 'Connection check succeeded. The rep counter is still a timer preview, not a sensor measurement.'} No motor commands are sent.</p>
        ${!session.mode ? `<section class="exercise-empty"><div class="exercise-empty-bars"><span></span><span></span><span></span></div><h2>Your session will appear here</h2><p>Select Flexion–Extension or Cycling to begin a guided UI trial.</p></section>` : `
        <div class="exercise-content">
          <div class="exercise-content-head"><div><p class="exercise-eyebrow">Current exercise</p><h2>${escapeHtml(selected)}</h2></div>${active ? `<span class="exercise-status ${running ? '' : 'muted'}"><i></i>${running ? 'In progress' : session.remaining > 0 ? 'Paused' : 'Complete'}</span>` : ''}</div>
          ${session.mode === 'flexion' && !session.flexionType ? `<div class="exercise-instruction"><h3>Choose Support or Resistance</h3><p>Your exercise details and repetition counter will appear here.</p></div>` : `
            ${session.mode === 'cycling' ? `<div class="exercise-video-wrap"><iframe id="exercise-video" title="Cycling exercise reference video" src="https://www.youtube-nocookie.com/embed/gC_L9qAHVJ8?enablejsapi=1&controls=0&mute=1&playsinline=1&rel=0" referrerpolicy="strict-origin-when-cross-origin" allow="autoplay; encrypted-media; picture-in-picture"></iframe>${!running ? `<div class="exercise-video-overlay">${cycleIcon}<p>${session.completed ? 'Session complete' : session.paused ? 'Session paused' : 'Ready when you are'}</p></div>` : ''}</div>` : ''}
            ${active ? `<div class="exercise-metrics"><article class="exercise-metric-card"><p>${session.mode === 'cycling' ? 'Resistance (UI sample)' : 'Exercise type'}</p><strong class="exercise-metric-word">${session.mode === 'cycling' ? '2x' : escapeHtml(session.activeType)}</strong></article><article class="exercise-metric-card"><p>Reps remaining</p><strong id="exercise-count" aria-live="polite">${session.remaining}</strong><div class="exercise-progress-track"><span id="exercise-bar" style="width:${Math.round((session.total - session.remaining) / session.total * 100)}%"></span></div></article></div>` : session.mode === 'cycling' ? `<div class="exercise-start-prompt"><p>Enter your repetitions to start the video and countdown.</p><button id="exercise-open-dialog" type="button">Enter ${arrowIcon}</button></div>` : ''}
            ${session.saving ? '<p class="exercise-notice">Saving session…</p>' : ''}${session.notice ? `<p class="exercise-notice success" role="status">${escapeHtml(session.notice)}</p>` : ''}${session.error ? `<p class="exercise-notice error" role="alert">${escapeHtml(session.error)}</p>${session.completed && !session.saved ? '<button id="exercise-retry-save" type="button" class="exercise-retry">Retry save</button>' : ''}` : ''}
            <section class="exercise-history"><div class="exercise-history-heading"><div><p class="exercise-eyebrow">Track record</p><h3>Previous exercises</h3></div><a href="/progress" data-link>See all →</a></div><div class="exercise-history-list">${historyRows()}</div></section>
          `}
        </div>`}
      </main>
      ${session.dialog ? `<div class="exercise-dialog-backdrop"><section class="exercise-dialog" role="dialog" aria-modal="true" aria-labelledby="exercise-dialog-title"><button type="button" id="exercise-close-dialog" class="exercise-dialog-close" aria-label="Close">×</button><p class="exercise-eyebrow">${escapeHtml(session.dialog)} session</p><h2 id="exercise-dialog-title">How many reps?</h2><form id="exercise-form"><label for="exercise-reps">Number of reps</label><input id="exercise-reps" name="reps" type="number" min="1" max="99" step="1" inputmode="numeric" required value="${escapeHtml(session.repInput)}" />${session.error ? `<p class="exercise-notice error">${escapeHtml(session.error)}</p>` : ''}<button type="submit">Start exercise ${arrowIcon}</button></form></section></div>` : ''}
    </div>`
  }

  function attach() {
    document.querySelectorAll('[data-mode]').forEach(button => button.addEventListener('click', () => selectMode(button.dataset.mode)))
    document.querySelectorAll('[data-flexion]').forEach(button => button.addEventListener('click', () => selectFlexion(button.dataset.flexion)))
    document.querySelector('#exercise-form')?.addEventListener('submit', begin)
    document.querySelector('#exercise-reps')?.addEventListener('input', event => { session.repInput = event.target.value })
    document.querySelector('#exercise-close-dialog')?.addEventListener('click', () => { session.dialog = null; session.error = ''; render() })
    document.querySelector('#exercise-open-dialog')?.addEventListener('click', () => { session.dialog = 'Cycling'; render() })
    document.querySelector('#exercise-toggle')?.addEventListener('click', togglePause)
    document.querySelector('#exercise-retry-save')?.addEventListener('click', () => void saveCompleted())
  }

  return { html, attach, reset }
}
