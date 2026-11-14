const root = document.querySelector('#app')
const state = {
  view: 'welcome', name: '', email: '', password: '', otp: ['', '', '', ''],
  user: null, busy: false, message: '', info: false, menu: false,
  connection: '', connectionError: false, connected: false, progress: []
}
const escapeHtml = value => String(value ?? '').replace(/[&<>"']/g, character => ({ '&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;', "'":'&#39;' }[character]))

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
  history.pushState({}, '', path)
  state.menu = false
  state.message = ''
  render()
  if (path === '/progress' && state.user) {
    api('/progress').then(result => { state.progress = result.sessions; if(location.pathname === '/progress')render() }).catch(()=>{})
  }
  scrollTo({ top: 0, behavior: 'instant' })
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
      <li><a href="/progress" data-link class="${p==='/progress'?'active':''}">Track Progress</a></li>
      <li><a href="/how-to" data-link class="${p==='/how-to'?'active':''}">How-to</a></li>
      <li><a href="/about" data-link class="${p==='/about'?'active':''}">About Us</a></li>
    </ul></nav>
    <button type="button" class="logout-btn top-logout" data-logout>Log Out ↗</button>
    <button class="hamburger" type="button" id="toggle-menu" aria-label="Toggle menu" aria-expanded="${state.menu}">${state.menu?'×':'☰'}</button>
    ${state.menu ? `<nav aria-label="Mobile navigation"><ul class="nav-list mobile-nav"><li><a data-link href="/dashboard">Home</a></li><li><a data-link href="/progress">Track Progress</a></li><li><a data-link href="/how-to">How-to</a></li><li><a data-link href="/about">About Us</a></li><li><button class="menu-logout" type="button" data-logout>Log Out ↗</button></li></ul></nav>`:''}
  </header>`
}
function dashboard() {
  return `${header()}<main class="dashboard-main"><h1 class="hero-title">Knee Rehabilitation</h1>
    <div class="hero-art"><div class="sage-band"></div><div class="photo-frame"><img src="/assets/rehabilitation.jpg" alt="Person receiving support during knee rehabilitation" /></div></div>
    <section class="hero-bottom"><div class="micro-label">DOULA: Knee Rehabilitation</div><h2>Start exercising</h2><p>Check if the rehabilitation device is connected</p>
      <button class="connect-btn" id="connect-btn" type="button" ${state.busy?'disabled':''}>${state.busy?'Checking…':'Connect Device'}</button>
      ${state.connection?`<p class="connect-status ${state.connectionError?'error':''}" role="status">${escapeHtml(state.connection)}</p>`:''}
    </section></main>`
}
function extraPage() {
  const path = location.pathname
  if (path === '/progress') return `${header()}<main class="inner-page"><div class="inner-eyebrow">DOULA: Rehabilitation records</div><h1 class="inner-title">Track Progress</h1><p class="inner-description">Your previous rehabilitation sessions will appear here once DOULA starts recording them.</p><section class="surface-panel"><h2>${state.progress.length?'Previous sessions':'No sessions recorded yet.'}</h2>${state.progress.length?`<table class="simple-table"><thead><tr><th>Date</th><th>Session</th></tr></thead><tbody>${state.progress.map((s,i)=>`<tr><td>${escapeHtml(s.date || '')}</td><td>${escapeHtml(s.label || 'Session '+(i+1))}</td></tr>`).join('')}</tbody></table>`:'<p>Complete a session with the device to start building your history.</p>'}</section><a href="/dashboard" class="inline-link" data-link>← Back to home</a></main>`
  if (path === '/how-to') return `${header()}<main class="inner-page"><div class="inner-eyebrow">DOULA: Guide</div><h1 class="inner-title">How-to</h1><p class="inner-description">A place for your DOULA device instructions and documents.</p><section class="surface-panel"><h2>Device guide</h2><p>The operating instructions will be added when the device workflow is finalized. Please use the device only with appropriate supervision.</p></section><a href="/dashboard" class="inline-link" data-link>← Back to home</a></main>`
  if (path === '/about') return `${header()}<main class="inner-page"><div class="inner-eyebrow">Meet DOULA</div><h1 class="inner-title">About Us</h1><p class="inner-description">DOULA is a prototype knee rehabilitation device with ESP32-based controllers.</p><section class="surface-panel"><h2>Our project</h2><p>More information about the DOULA team and the device will be added here.</p></section><a href="/dashboard" class="inline-link" data-link>← Back to home</a></main>`
  if (path === '/device') return `${header()}<main class="inner-page"><div class="inner-eyebrow">DOULA: Device</div><h1 class="inner-title">Device connection</h1><p class="inner-description">${state.connected ? 'The ESP32 coordinator responded to the connection check. This page will contain your exercise and monitoring controls once we have the device protocol.' : 'Please use Connect Device on the dashboard to check the ESP32.'}</p><div class="surface-panel">${state.connected?'<span class="pill-status">● ESP32 responded</span>':'<h2>Device not checked yet</h2>'}<p style="margin-top:16px">No movement commands or motor controls are sent from this prototype page.</p></div><a href="/dashboard" class="inline-link" data-link>← Back to home</a></main>`
  return dashboard()
}
function render() {
  root.innerHTML = state.user ? (location.pathname === '/dashboard' || location.pathname === '/' ? dashboard() : extraPage()) : authPage()
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
      state.user=result.user;state.password='';navigate('/dashboard')
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
      state.user=result.user;state.password='';navigate('/dashboard')
    })
  })
  document.querySelector('#toggle-menu')?.addEventListener('click',()=>{state.menu=!state.menu;render()})
  document.querySelectorAll('[data-link]').forEach(el=>el.addEventListener('click',e=>{e.preventDefault();navigate(el.getAttribute('href'))}))
  document.querySelectorAll('[data-logout]').forEach(el=>el.addEventListener('click',()=>withPending(async()=>{
    await api('/auth/logout',{method:'POST'});state.user=null;state.view='login';state.email='';state.password='';state.menu=false;state.connected=false;state.connection='';navigate('/')
  })))
  document.querySelector('#connect-btn')?.addEventListener('click',()=>withPending(async()=>{
    const result=await api('/device/status')
    if(result.connected){state.connected=true;state.connection='';navigate('/device')}
    else{state.connection=result.message;state.connectionError=true}
  }))
}
window.addEventListener('popstate',()=>render())
;(async()=>{
  try {
    const result=await api('/me')
    state.user=result.user
    if(state.user&&location.pathname==='/progress'){
      const progress=await api('/progress');state.progress=progress.sessions
    }
    if(!state.user&&location.pathname!=='/') history.replaceState({},'','/')
  } catch { state.user=null }
  render()
})()
