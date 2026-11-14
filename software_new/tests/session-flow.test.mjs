import test from 'node:test'
import assert from 'node:assert/strict'
import http from 'node:http'
import { createHash } from 'node:crypto'
import { mkdtemp, rm } from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { spawn } from 'node:child_process'
import { createAppServer } from '../server.mjs'

const testsDir = path.dirname(fileURLToPath(import.meta.url))
const appRoot = path.resolve(testsDir, '..')

async function listen(srv) {
  await new Promise(resolve => srv.listen(0, '127.0.0.1', resolve))
  const addr = srv.address()
  return `http://127.0.0.1:${addr.port}`
}

function closeServer(srv) {
  srv.closeAllConnections?.()
  return new Promise(resolve => srv.close(resolve))
}

async function createLoggedInCookie(baseUrl, store, email = 'patient@gmail.com', password = 'password123') {
  const { code } = store.startSignup('Test Patient', email)
  const verifyRes = await fetch(`${baseUrl}/api/auth/verify`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email, code }),
  })
  assert.equal(verifyRes.status, 200)
  const verifiedCookie = verifyRes.headers.getSetCookie()[0].split(';')[0]

  const finishRes = await fetch(`${baseUrl}/api/auth/finish`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', Cookie: verifiedCookie },
    body: JSON.stringify({ password }),
  })
  assert.equal(finishRes.status, 200)
  const sessionCookie = finishRes.headers.getSetCookie().find(c => c.startsWith('doula_session=')).split(';')[0]
  return { sessionCookie, finishData: await finishRes.json() }
}

test('bypass mode marks ESP bypassed and serves ../software session page', async () => {
  const dataDir = await mkdtemp(path.join(testsDir, '.tmp-bypass-'))
  const { server, store } = await createAppServer({ dataDir, bypassEsp: true })
  const baseUrl = await listen(server)
  try {
    const unauthSession = await fetch(`${baseUrl}/session/`, { redirect: 'manual' })
    assert.equal(unauthSession.status, 302)
    assert.equal(unauthSession.headers.get('location'), '/')

    const { sessionCookie, finishData } = await createLoggedInCookie(baseUrl, store)
    assert.equal(finishData.espBypassed, true)
    assert.equal(finishData.deviceVerified, true)

    const meRes = await fetch(`${baseUrl}/api/me`, { headers: { Cookie: sessionCookie } })
    const me = await meRes.json()
    assert.equal(me.espBypassed, true)
    assert.equal(me.deviceVerified, true)

    const statusRes = await fetch(`${baseUrl}/api/device/status`, { headers: { Cookie: sessionCookie } })
    const status = await statusRes.json()
    assert.equal(status.connected, true)
    assert.equal(status.bypassed, true)

    const redirectSession = await fetch(`${baseUrl}/session`, { headers: { Cookie: sessionCookie }, redirect: 'manual' })
    assert.equal(redirectSession.status, 302)
    assert.equal(redirectSession.headers.get('location'), '/session/')

    const htmlRes = await fetch(`${baseUrl}/session/`, { headers: { Cookie: sessionCookie } })
    assert.equal(htmlRes.status, 200)
    assert.match(htmlRes.headers.get('content-type'), /^text\/html/)
    const html = await htmlRes.text()
    assert.match(html, /Cycling Dashboard/)
    assert.match(html, /href="session\.css"/)
    assert.match(html, /src="session\.js"/)

    const cssRes = await fetch(`${baseUrl}/session/session.css`, { headers: { Cookie: sessionCookie } })
    assert.equal(cssRes.status, 200)
    assert.match(cssRes.headers.get('content-type'), /^text\/css/)

    const jsRes = await fetch(`${baseUrl}/session/session.js`, { headers: { Cookie: sessionCookie } })
    assert.equal(jsRes.status, 200)
    assert.match(jsRes.headers.get('content-type'), /^text\/javascript/)

    // %2f prevents URL dot-segment normalization so serveStatic's traversal guard is directly exercised.
    const urlParts = new URL(baseUrl)
    const traversalStatus = await new Promise((resolve, reject) => {
      const req = http.request({
        hostname: urlParts.hostname,
        port: urlParts.port,
        path: '/session/..%2fsoftware_new/server.mjs',
        headers: { Cookie: sessionCookie },
      }, res => {
        res.resume()
        resolve(res.statusCode)
      })
      req.on('error', reject)
      req.end()
    })
    assert.equal(traversalStatus, 404)

    const dirRes = await fetch(`${baseUrl}/session/videos/`, { headers: { Cookie: sessionCookie } })
    assert.equal(dirRes.status, 404)
  } finally {
    await closeServer(server)
    await rm(dataDir, { recursive: true, force: true })
  }
})

test('normal mode gates /session/ until ESP WebSocket check succeeds', async () => {
  const dataDir = await mkdtemp(path.join(testsDir, '.tmp-normal-'))

  // Minimal WebSocket echo/handshake mock representing U1 ESP_MASTER.
  const wsSockets = new Set()
  const wsMock = http.createServer()
  wsMock.on('upgrade', (req, socket) => {
    wsSockets.add(socket)
    socket.on('close', () => wsSockets.delete(socket))
    const key = req.headers['sec-websocket-key']
    const accept = createHash('sha1').update(key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest('base64')
    socket.write(
      'HTTP/1.1 101 Switching Protocols\r\n' +
      'Upgrade: websocket\r\n' +
      'Connection: Upgrade\r\n' +
      `Sec-WebSocket-Accept: ${accept}\r\n\r\n`
    )
  })
  const wsHttpUrl = await listen(wsMock)
  const wsUrl = wsHttpUrl.replace('http://', 'ws://')

  const { server, store } = await createAppServer({ dataDir, bypassEsp: false, wsUrl })
  const baseUrl = await listen(server)
  try {
    const { sessionCookie, finishData } = await createLoggedInCookie(baseUrl, store, 'normal@gmail.com')
    assert.equal(finishData.espBypassed, false)
    assert.equal(finishData.deviceVerified, false)

    // Blocked before ESP check passes.
    const gated = await fetch(`${baseUrl}/session/`, { headers: { Cookie: sessionCookie }, redirect: 'manual' })
    assert.equal(gated.status, 302)
    assert.equal(gated.headers.get('location'), '/dashboard')

    const gatedAsset = await fetch(`${baseUrl}/session/session.css`, { headers: { Cookie: sessionCookie } })
    assert.equal(gatedAsset.status, 403)

    // ESP check passes against mock WebSocket server.
    const statusRes = await fetch(`${baseUrl}/api/device/status`, { headers: { Cookie: sessionCookie } })
    const status = await statusRes.json()
    assert.equal(status.connected, true)
    assert.equal(status.bypassed, false)

    const meRes = await fetch(`${baseUrl}/api/me`, { headers: { Cookie: sessionCookie } })
    const me = await meRes.json()
    assert.equal(me.espBypassed, false)
    assert.equal(me.deviceVerified, true)

    // Now /session/ is unlocked.
    const unlocked = await fetch(`${baseUrl}/session/`, { headers: { Cookie: sessionCookie } })
    assert.equal(unlocked.status, 200)
    assert.match(await unlocked.text(), /Cycling Dashboard/)

    // Logout revokes session verification.
    await fetch(`${baseUrl}/api/auth/logout`, { method: 'POST', headers: { Cookie: sessionCookie } })
    const afterLogout = await fetch(`${baseUrl}/session/`, { headers: { Cookie: sessionCookie }, redirect: 'manual' })
    assert.equal(afterLogout.status, 302)
    assert.equal(afterLogout.headers.get('location'), '/')
  } finally {
    for (const s of wsSockets) s.destroy()
    await closeServer(server)
    await closeServer(wsMock)
    await rm(dataDir, { recursive: true, force: true })
  }
})

test('CLI flag --bypass-esp enables bypass mode on startup', async () => {
  const child = spawn(process.execPath, ['server.mjs', '--bypass-esp'], {
    cwd: appRoot,
    env: { ...process.env, NODE_ENV: 'development', PORT: '0', HOST: '127.0.0.1' },
    stdio: ['ignore', 'pipe', 'pipe'],
  })

  try {
    const output = await new Promise((resolve, reject) => {
      let buf = ''
      const timer = setTimeout(() => reject(new Error(`Server startup timed out. Output:\n${buf}`)), 5000)
      child.stdout.on('data', chunk => {
        buf += chunk.toString()
        if (buf.includes('ESP32: BYPASSED')) {
          clearTimeout(timer)
          resolve(buf)
        }
      })
      child.stderr.on('data', chunk => { buf += chunk.toString() })
      child.on('exit', code => {
        clearTimeout(timer)
        reject(new Error(`Server exited early (${code}). Output:\n${buf}`))
      })
    })
    assert.match(output, /ESP32: BYPASSED for testing/)
  } finally {
    child.kill()
  }
})
