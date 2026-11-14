import http from 'node:http'
import { readFile } from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { AuthStore } from './backend/core.mjs'
import { sendVerificationEmail } from './backend/email.mjs'

const root = path.dirname(fileURLToPath(import.meta.url))
// Load optional .env without another package.
try {
  const envText = await readFile(path.join(root, '.env'), 'utf8')
  for (const line of envText.split(/\r?\n/)) {
    const match = /^([A-Z_][A-Z0-9_]*)=(.*)$/.exec(line.trim())
    if (match && !process.env[match[1]]) process.env[match[1]] = match[2].replace(/^['"]|['"]$/g, '')
  }
} catch (e) { if (e.code !== 'ENOENT') throw e }

const config = {
  host: process.env.HOST || '127.0.0.1',
  port: Number(process.env.PORT || 3000),
  smtp: { user: process.env.SMTP_USER || '', password: process.env.SMTP_APP_PASSWORD || '' },
  ws: process.env.ESP32_WS_URL || '',
}
const store = new AuthStore(path.join(root, 'backend', 'data'))
await store.init()

function json(res, status, value, extraHeaders = {}) {
  res.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-store', ...extraHeaders })
  res.end(JSON.stringify(value))
}
function parseCookies(req) {
  return Object.fromEntries((req.headers.cookie || '').split(';').map(c => c.trim()).filter(Boolean).map(c => {
    const pos = c.indexOf('=')
    return [c.slice(0, pos), decodeURIComponent(c.slice(pos + 1))]
  }))
}
function cookie(name, value, maxAge = 60 * 60 * 24 * 7) {
  return `${name}=${encodeURIComponent(value)}; HttpOnly; SameSite=Lax; Path=/; Max-Age=${maxAge}${process.env.COOKIE_SECURE === 'true' ? '; Secure' : ''}`
}
async function readBody(req) {
  let text = ''
  for await (const chunk of req) {
    text += chunk.toString('utf8')
    if (text.length > 20_000) throw new Error('Request too large.')
  }
  try { return JSON.parse(text || '{}') }
  catch { throw new Error('Invalid request.') }
}
function loggedIn(req) { return store.getSession(parseCookies(req).doula_session) }
function responseError(res, e) { json(res, 400, { ok: false, message: e.message || 'Something went wrong.' }) }
function checkEsp32(url) {
  return new Promise(resolve => {
    if (!url) return resolve({ connected: false, message: 'ESP32 address has not been configured yet.' })
    if (!/^wss?:\/\//.test(url)) return resolve({ connected: false, message: 'ESP32_WS_URL must start with ws:// or wss://.' })
    let ws
    let finished = false
    const timeout = setTimeout(() => finish(false, 'ESP32 did not respond. Check Wi-Fi and power.'), 3000)
    function finish(connected, message) {
      if (finished) return
      finished = true
      clearTimeout(timeout)
      if (ws) { try { ws.close() } catch {} }
      resolve({ connected, message })
    }
    try {
      ws = new WebSocket(url)
      ws.addEventListener('open', () => finish(true, 'ESP32 is responding.'))
      ws.addEventListener('error', () => finish(false, 'Cannot reach ESP32. Check its address, Wi-Fi and firmware.'))
    } catch { finish(false, 'Could not open the ESP32 WebSocket connection.') }
  })
}
const contentType = { '.html': 'text/html', '.css': 'text/css', '.js': 'text/javascript', '.jpg': 'image/jpeg', '.jpeg': 'image/jpeg', '.png': 'image/png', '.svg': 'image/svg+xml', '.ico': 'image/x-icon' }

export const server = http.createServer(async (req, res) => {
  const pathname = new URL(req.url || '/', `http://${req.headers.host || 'localhost'}`).pathname
  try {
    if (pathname.startsWith('/api/')) {
      const method = req.method
      if (method === 'GET' && pathname === '/api/me') return json(res, 200, { ok: true, user: loggedIn(req) })
      if (method === 'POST' && pathname === '/api/auth/start') {
        const { name, email } = await readBody(req)
        const signup = store.startSignup(name, email)
        try {
          const delivery = await sendVerificationEmail(signup.email, signup.code, config.smtp)
          return json(res, 200, { ok: true, delivery, message: delivery === 'email' ? 'A four-digit OTP has been sent to your Gmail.' : 'For this local demo, find your four-digit OTP in the server terminal.' })
        } catch (error) {
          store.signups.delete(signup.email)
          console.error('Email delivery error:', error.message)
          return json(res, 502, { ok: false, message: 'Could not send the OTP email. Check Gmail SMTP settings.' })
        }
      }
      if (method === 'POST' && pathname === '/api/auth/verify') {
        const { email, code } = await readBody(req)
        const ticket = store.verifySignup(email, code)
        return json(res, 200, { ok: true }, { 'Set-Cookie': cookie('doula_verified', ticket, 60 * 60) })
      }
      if (method === 'POST' && pathname === '/api/auth/finish') {
        const { password } = await readBody(req)
        const ticket = parseCookies(req).doula_verified
        const { token, user } = await store.finishSignup(ticket, password)
        return json(res, 200, { ok: true, user }, { 'Set-Cookie': [cookie('doula_verified', '', 0), cookie('doula_session', token)] })
      }
      if (method === 'POST' && pathname === '/api/auth/login') {
        const { email, password } = await readBody(req)
        const { token, user } = await store.login(email, password)
        return json(res, 200, { ok: true, user }, { 'Set-Cookie': cookie('doula_session', token) })
      }
      if (method === 'POST' && pathname === '/api/auth/logout') {
        store.logout(parseCookies(req).doula_session)
        return json(res, 200, { ok: true }, { 'Set-Cookie': cookie('doula_session', '', 0) })
      }
      if (method === 'GET' && pathname === '/api/device/status') {
        if (!loggedIn(req)) return json(res, 401, { ok: false, message: 'Please log in.' })
        const result = await checkEsp32(config.ws)
        return json(res, 200, { ok: true, ...result })
      }
      if (method === 'GET' && pathname === '/api/progress') {
        const user = loggedIn(req)
        if (!user) return json(res, 401, { ok: false, message: 'Please log in.' })
        return json(res, 200, { ok: true, sessions: store.getProgress(user.id) })
      }
      return json(res, 404, { ok: false, message: 'API route not found.' })
    }

    // Serve the single-page website (including refreshes of /dashboard, /progress etc.).
    const assetPath = pathname.startsWith('/assets/') ? path.join(root, 'public', pathname) : pathname === '/styles.css' || pathname === '/app.js' ? path.join(root, 'public', pathname) : path.join(root, 'public', 'index.html')
    const safeRoot = path.join(root, 'public') + path.sep
    if (!assetPath.startsWith(safeRoot)) return json(res, 404, { ok: false })
    const body = await readFile(assetPath)
    res.writeHead(200, { 'Content-Type': contentType[path.extname(assetPath)] || 'application/octet-stream' })
    res.end(body)
  } catch (e) {
    if (pathname.startsWith('/api/')) return responseError(res, e)
    json(res, 404, { ok: false, message: 'Page not found.' })
  }
})

if (process.env.NODE_ENV !== 'test') {
  server.listen(config.port, config.host, () => {
    console.log(`\nDOULA website: http://${config.host}:${config.port}`)
    console.log(config.smtp.user && config.smtp.password ? 'OTP delivery: Gmail SMTP configured.' : 'OTP delivery: LOCAL DEMO (code printed in this terminal).')
    console.log(config.ws ? 'ESP32 configured.' : 'ESP32: not configured yet (Connect Device will show a clear message).')
  })
}
