import test from 'node:test'
import assert from 'node:assert/strict'
import { mkdtemp, readFile, rm } from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { createAppServer } from '../server.mjs'

const here = path.dirname(fileURLToPath(import.meta.url))

function extractCookies(response, jar) {
  const raw = response.headers.getSetCookie ? response.headers.getSetCookie() : []
  for (const header of raw) {
    const [pair] = header.split(';')
    const eq = pair.indexOf('=')
    if (eq === -1) continue
    const name = pair.slice(0, eq).trim()
    const val = pair.slice(eq + 1).trim()
    if (!val) delete jar[name]
    else jar[name] = val
  }
}

function cookieHeader(jar) {
  return Object.entries(jar).map(([k, v]) => `${k}=${v}`).join('; ')
}

async function signupAndLogin(baseUrl, store, name, email, password) {
  const jar = {}
  const signup = store.startSignup(name, email)
  const verifyRes = await fetch(`${baseUrl}/api/auth/verify`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email: signup.email, code: signup.code }),
  })
  assert.equal(verifyRes.status, 200)
  extractCookies(verifyRes, jar)

  const finishRes = await fetch(`${baseUrl}/api/auth/finish`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', Cookie: cookieHeader(jar) },
    body: JSON.stringify({ password }),
  })
  assert.equal(finishRes.status, 200)
  extractCookies(finishRes, jar)
  return jar
}

test('master workflow: dashboard -> session selector -> cycling & flexion -> progress.json persistence', async () => {
  const dataDir = await mkdtemp(path.join(here, '.tmp-master-flow-'))
  const { server, store } = await createAppServer({ host: '127.0.0.1', port: 0, dataDir, bypassEsp: false })
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve))
  const { port } = server.address()
  const baseUrl = `http://127.0.0.1:${port}`

  try {
    // 1. Unauthenticated /device/ redirects to /
    const anonRes = await fetch(`${baseUrl}/device/`, { redirect: 'manual' })
    assert.equal(anonRes.status, 302)
    assert.equal(anonRes.headers.get('location'), '/')

    const jar = await signupAndLogin(baseUrl, store, 'Tester', 'tester@gmail.com', 'password123')

    // 2. Without demo or connected ESP32, /device/ redirects to /dashboard and /device/session-select.css is 403
    const blockedRes = await fetch(`${baseUrl}/device/`, { headers: { Cookie: cookieHeader(jar) }, redirect: 'manual' })
    assert.equal(blockedRes.status, 302)
    assert.equal(blockedRes.headers.get('location'), '/dashboard')

    const blockedAsset = await fetch(`${baseUrl}/device/session-select.css`, { headers: { Cookie: cookieHeader(jar) } })
    assert.equal(blockedAsset.status, 403)

    // 3. Enter Demo mode via POST /api/device/demo
    const demoRes = await fetch(`${baseUrl}/api/device/demo`, {
      method: 'POST',
      headers: { Cookie: cookieHeader(jar) },
    })
    assert.equal(demoRes.status, 200)
    const demoData = await demoRes.json()
    assert.equal(demoData.connectionMode, 'demo')

    const meRes = await fetch(`${baseUrl}/api/me`, { headers: { Cookie: cookieHeader(jar) } })
    assert.equal((await meRes.json()).connectionMode, 'demo')

    // 4. /device redirects to /device/ and serves Session Selector from b1 + See Progress link
    const deviceRes = await fetch(`${baseUrl}/device`, { headers: { Cookie: cookieHeader(jar) }, redirect: 'manual' })
    assert.equal(deviceRes.status, 302)
    assert.equal(deviceRes.headers.get('location'), '/device/')

    const selectorRes = await fetch(`${baseUrl}/device/`, { headers: { Cookie: cookieHeader(jar) } })
    assert.equal(selectorRes.status, 200)
    const selectorHtml = await selectorRes.text()
    assert.match(selectorHtml, /Cycling/)
    assert.match(selectorHtml, /Flexion/)
    assert.match(selectorHtml, /href="\/progress"/)

    // 5. Cycling page serves 4-mode selector + HUD + Pause/Resume + End Session + See Progress
    const cyclingRes = await fetch(`${baseUrl}/device/cycling/`, { headers: { Cookie: cookieHeader(jar) } })
    assert.equal(cyclingRes.status, 200)
    const cyclingHtml = await cyclingRes.text()
    assert.match(cyclingHtml, /id="cyclingSetupView"/)
    assert.match(cyclingHtml, /Constant Torque/)
    assert.match(cyclingHtml, /Constant Power/)
    assert.match(cyclingHtml, /Isokinetic Cadence/)
    assert.match(cyclingHtml, /Manual Control/)
    assert.match(cyclingHtml, /id="startCyclingBtn"/)
    assert.match(cyclingHtml, /id="pauseResumeBtn"/)
    assert.match(cyclingHtml, /id="endSessionBtn"/)
    assert.match(cyclingHtml, /href="\/progress"/)

    // 6. Flexion page serves modes + Pause + End Session + See Progress + master typography
    const flexionRes = await fetch(`${baseUrl}/device/flexion/`, { headers: { Cookie: cookieHeader(jar) } })
    assert.equal(flexionRes.status, 200)
    const flexionHtml = await flexionRes.text()
    assert.match(flexionHtml, /Passive Assist/)
    assert.match(flexionHtml, /Isotonic Mode/)
    assert.match(flexionHtml, /Isokinetic Mode/)
    assert.match(flexionHtml, /min="120"/)
    assert.match(flexionHtml, /value="180"/)
    assert.match(flexionHtml, /120°–180°/)
    assert.match(flexionHtml, /5–45 N·m/)
    assert.match(flexionHtml, /10–45 N·m/)
    assert.match(flexionHtml, /id="setupWarningBanner"/)
    assert.match(flexionHtml, /id="beginExerciseBtn"/)
    assert.match(flexionHtml, /id="parkHomeBtn"/)
    assert.match(flexionHtml, /id="pauseSessionBtn"/)
    assert.match(flexionHtml, /id="endSessionBtn"/)
    assert.match(flexionHtml, /href="\/progress"/)

    const flexionCssRes = await fetch(`${baseUrl}/device/flexion/flexion.css`, { headers: { Cookie: cookieHeader(jar) } })
    assert.equal(flexionCssRes.status, 200)
    const flexionCss = await flexionCssRes.text()
    assert.match(flexionCss, /DM Serif Display/)
    assert.match(flexionCss, /Manrope/)
    assert.match(flexionCss, /#3f580c/)

    // 7. Missing static asset & path traversal never expose server files
    const missingAsset = await fetch(`${baseUrl}/device/does-not-exist.js`, { headers: { Cookie: cookieHeader(jar) } })
    assert.equal(missingAsset.status, 404)

    const traversalRes = await fetch(`${baseUrl}/device/%2e%2e/software_new/server.mjs`, { headers: { Cookie: cookieHeader(jar) } })
    const traversalBody = await traversalRes.text()
    assert.doesNotMatch(traversalBody, /createAppServer/)

    // 8. Saving Cycling and Flexion sessions persists into progress.json
    const saveCycling = await fetch(`${baseUrl}/api/progress`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', Cookie: cookieHeader(jar) },
      body: JSON.stringify({ type: 'Cycling', mode: 'Cycling', torqueNm: 112.4, reps: 14, durationSeconds: 45, source: 'demo' }),
    })
    assert.equal(saveCycling.status, 201)

    const savePassive = await fetch(`${baseUrl}/api/progress`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', Cookie: cookieHeader(jar) },
      body: JSON.stringify({ type: 'Support', mode: 'Passive Assist Mode', torqueNm: 12, reps: 15, durationSeconds: 52, source: 'demo' }),
    })
    assert.equal(savePassive.status, 201)

    const saveIsotonic = await fetch(`${baseUrl}/api/progress`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', Cookie: cookieHeader(jar) },
      body: JSON.stringify({ type: 'Resistance', mode: 'Physiological Curve-Shaped Isotonic', torqueNm: 26, reps: 12, durationSeconds: 44, source: 'demo' }),
    })
    assert.equal(saveIsotonic.status, 201)

    const progressRes = await fetch(`${baseUrl}/api/progress`, { headers: { Cookie: cookieHeader(jar) } })
    assert.equal(progressRes.status, 200)
    const progressData = await progressRes.json()
    assert.equal(progressData.sessions.length, 3)
    assert.equal(progressData.sessions[0].mode, 'Physiological Curve-Shaped Isotonic')
    assert.equal(progressData.sessions[1].mode, 'Passive Assist Mode')
    assert.equal(progressData.sessions[2].type, 'Cycling')

    const onDisk = JSON.parse(await readFile(path.join(dataDir, 'progress.json'), 'utf8'))
    assert.equal(onDisk.length, 3)
  } finally {
    await new Promise(resolve => server.close(resolve))
    await rm(dataDir, { recursive: true, force: true })
  }
})

test('bypassed or connected ESP32 mode unlocks /device/ as connected', async () => {
  const dataDir = await mkdtemp(path.join(here, '.tmp-master-conn-'))
  const { server, store } = await createAppServer({ host: '127.0.0.1', port: 0, dataDir, bypassEsp: true })
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve))
  const { port } = server.address()
  const baseUrl = `http://127.0.0.1:${port}`

  try {
    const jar = await signupAndLogin(baseUrl, store, 'Conn', 'conn@gmail.com', 'password123')
    const statusRes = await fetch(`${baseUrl}/api/device/status`, { headers: { Cookie: cookieHeader(jar) } })
    assert.equal(statusRes.status, 200)
    assert.equal((await statusRes.json()).connected, true)

    const cfgRes = await fetch(`${baseUrl}/api/device/config`, { headers: { Cookie: cookieHeader(jar) } })
    const cfg = await cfgRes.json()
    assert.equal(cfg.deviceMode, 'connected')
    assert.equal(cfg.bypassEsp, true)

    const selectorRes = await fetch(`${baseUrl}/device/`, { headers: { Cookie: cookieHeader(jar) } })
    assert.equal(selectorRes.status, 200)
  } finally {
    await new Promise(resolve => server.close(resolve))
    await rm(dataDir, { recursive: true, force: true })
  }
})
