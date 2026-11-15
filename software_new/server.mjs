import http from 'node:http'
import { readFile, stat } from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { AuthStore } from './backend/core.mjs'
import { computeClinicalPrescription } from './backend/clinical.mjs'
import { sendVerificationEmail } from './backend/email.mjs'
import { attachWsBridge, isAllowedLanWsUrl, normalizeEspWsUrl } from './backend/wsBridge.mjs'

const root = path.dirname(fileURLToPath(import.meta.url))
// Load optional .env without another package.
try {
  const envText = await readFile(path.join(root, '.env'), 'utf8')
  for (const line of envText.split(/\r?\n/)) {
    const match = /^([A-Z_][A-Z0-9_]*)=(.*)$/.exec(line.trim())
    if (match && !process.env[match[1]]) process.env[match[1]] = match[2].replace(/^['"]|['"]$/g, '')
  }
} catch (e) { if (e.code !== 'ENOENT') throw e }

const defaultBypass =
  process.argv.includes('--bypass-esp') ||
  process.argv.includes('--bypass') ||
  /^(1|true|yes)$/i.test(process.env.ESP32_BYPASS || '')

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
const contentType = {
  '.html': 'text/html; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.jpg': 'image/jpeg',
  '.jpeg': 'image/jpeg',
  '.png': 'image/png',
  '.svg': 'image/svg+xml',
  '.ico': 'image/x-icon',
  '.mp4': 'video/mp4',
  '.webm': 'video/webm',
}

async function serveStatic(res, baseDir, relativePath) {
  let decoded
  try { decoded = decodeURIComponent(relativePath) }
  catch { return json(res, 404, { ok: false, message: 'Page not found.' }) }
  const target = path.resolve(baseDir, decoded.replace(/^\/+/, ''))
  const rel = path.relative(baseDir, target)
  if (!rel || rel.startsWith('..') || path.isAbsolute(rel)) {
    return json(res, 404, { ok: false, message: 'Page not found.' })
  }
  const info = await stat(target)
  if (!info.isFile()) return json(res, 404, { ok: false, message: 'Page not found.' })
  const body = await readFile(target)
  res.writeHead(200, { 'Content-Type': contentType[path.extname(target).toLowerCase()] || 'application/octet-stream' })
  res.end(body)
}

export async function createAppServer(options = {}) {
  const config = {
    host: options.host ?? (process.env.HOST || '127.0.0.1'),
    port: options.port ?? Number(process.env.PORT || 3000),
    smtp: options.smtp ?? { user: process.env.SMTP_USER || '', password: process.env.SMTP_APP_PASSWORD || '' },
    ws: options.wsUrl ?? (process.env.ESP32_WS_URL || ''),
    bypassEsp: options.bypassEsp ?? defaultBypass,
  }
  const espChecker = options.checkEsp32 ?? checkEsp32
  const publicDir = path.resolve(options.publicDir || path.join(root, 'public'))
  const softwareDir = path.resolve(options.softwareDir || path.resolve(root, '..', 'software'))
  const store = new AuthStore(options.dataDir || path.join(root, 'backend', 'data'))
  await store.init()

  // Tracks each login session's connection mode ('demo' or 'connected') for /device/* routes.
  let runtimeWsUrl = config.ws
  const sessionModes = new Map()
  const resolveDeviceMode = token => {
    if (!token) return 'none'
    const mode = sessionModes.get(token)
    if (mode === 'demo' || mode === 'connected') return mode
    return config.bypassEsp ? 'connected' : 'none'
  }

  const appServer = http.createServer(async (req, res) => {
    const reqUrl = new URL(req.url || '/', `http://${req.headers.host || 'localhost'}`)
    const pathname = reqUrl.pathname
    try {
      const token = parseCookies(req).doula_session
      const user = store.getSession(token)

      if (pathname.startsWith('/api/')) {
        const method = req.method
        if (method === 'GET' && pathname === '/api/me') {
          return json(res, 200, {
            ok: true,
            user,
            connectionMode: user ? resolveDeviceMode(token) : 'none',
            espUrl: runtimeWsUrl,
          })
        }
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
          const { token: newToken, user: createdUser } = await store.finishSignup(ticket, password)
          return json(
            res,
            200,
            { ok: true, user: createdUser, connectionMode: config.bypassEsp ? 'connected' : 'none' },
            { 'Set-Cookie': [cookie('doula_verified', '', 0), cookie('doula_session', newToken)] }
          )
        }
        if (method === 'POST' && pathname === '/api/auth/login') {
          const { email, password } = await readBody(req)
          const { token: newToken, user: loggedUser } = await store.login(email, password)
          return json(
            res,
            200,
            { ok: true, user: loggedUser, connectionMode: config.bypassEsp ? 'connected' : 'none' },
            { 'Set-Cookie': cookie('doula_session', newToken) }
          )
        }
        if (method === 'POST' && pathname === '/api/auth/logout') {
          store.logout(token)
          sessionModes.delete(token)
          return json(res, 200, { ok: true }, { 'Set-Cookie': cookie('doula_session', '', 0) })
        }
        if (method === 'POST' && pathname === '/api/device/demo') {
          if (!user) return json(res, 401, { ok: false, message: 'Please log in.' })
          sessionModes.set(token, 'demo')
          return json(res, 200, { ok: true, connectionMode: 'demo' })
        }
        if (method === 'GET' && pathname === '/api/device/status') {
          if (!user) return json(res, 401, { ok: false, message: 'Please log in.' })
          if (config.bypassEsp && !reqUrl.searchParams.get('ws')) {
            sessionModes.set(token, 'connected')
            return json(res, 200, { ok: true, connected: true, bypassed: true, wsUrl: runtimeWsUrl, message: 'ESP32 check bypassed for testing.' })
          }
          const rawOverride = reqUrl.searchParams.get('ws')
          let targetUrl = runtimeWsUrl
          if (rawOverride) {
            targetUrl = normalizeEspWsUrl(rawOverride)
            if (!isAllowedLanWsUrl(targetUrl)) {
              return json(res, 400, { ok: false, connected: false, message: 'ESP32 URL must point to a local/LAN IP (192.168.x.x, 10.x.x.x, .local, or localhost).' })
            }
          }
          const result = await espChecker(targetUrl)
          if (result.connected) {
            if (targetUrl !== runtimeWsUrl) {
              runtimeWsUrl = targetUrl
              wsBridge.resetUpstream()
            }
            sessionModes.set(token, 'connected')
          }
          return json(res, 200, { ok: true, bypassed: false, wsUrl: targetUrl, ...result })
        }
        if (method === 'POST' && pathname === '/api/device/connect') {
          if (!user) return json(res, 401, { ok: false, message: 'Please log in.' })
          const body = await readBody(req)
          const rawInput = body?.wsUrl ? String(body.wsUrl).trim() : ''
          if (config.bypassEsp && !rawInput) {
            sessionModes.set(token, 'connected')
            return json(res, 200, { ok: true, connected: true, bypassed: true, wsUrl: runtimeWsUrl, message: 'ESP32 check bypassed for testing.' })
          }
          const targetUrl = rawInput ? normalizeEspWsUrl(rawInput) : runtimeWsUrl
          if (rawInput && !isAllowedLanWsUrl(targetUrl)) {
            return json(res, 400, { ok: false, connected: false, message: 'ESP32 URL must point to a local/LAN IP (192.168.x.x, 10.x.x.x, .local, or localhost).' })
          }
          const result = await espChecker(targetUrl)
          if (result.connected) {
            if (targetUrl !== runtimeWsUrl) {
              runtimeWsUrl = targetUrl
              wsBridge.resetUpstream()
            }
            sessionModes.set(token, 'connected')
          }
          return json(res, 200, { ok: true, bypassed: false, wsUrl: targetUrl, ...result })
        }
        if (method === 'GET' && pathname === '/api/device/config') {
          if (!user) return json(res, 401, { ok: false, message: 'Please log in.' })
          return json(res, 200, {
            ok: true,
            bypassEsp: config.bypassEsp,
            wsUrl: runtimeWsUrl,
            bridgeWsPath: '/ws/device',
            deviceMode: resolveDeviceMode(token),
            clinicalIntake: user.clinicalIntake || null,
            prescription: user.prescription || null,
          })
        }
        if (method === 'POST' && pathname === '/api/clinical/preview') {
          if (!user) return json(res, 401, { ok: false, message: 'Please log in.' })
          const prescription = computeClinicalPrescription(await readBody(req))
          return json(res, 200, { ok: true, prescription })
        }
        if (method === 'POST' && pathname === '/api/clinical/intake') {
          if (!user) return json(res, 401, { ok: false, message: 'Please log in.' })
          const updatedUser = await store.setClinicalIntake(user.id, await readBody(req))
          return json(res, 200, {
            ok: true,
            user: updatedUser,
            clinicalIntake: updatedUser.clinicalIntake,
            prescription: updatedUser.prescription,
          })
        }
        if (method === 'GET' && pathname === '/api/clinical/prescription') {
          if (!user) return json(res, 401, { ok: false, message: 'Please log in.' })
          return json(res, 200, {
            ok: true,
            clinicalIntake: user.clinicalIntake || null,
            prescription: user.prescription || null,
          })
        }
        if (method === 'GET' && pathname === '/api/progress') {
          if (!user) return json(res, 401, { ok: false, message: 'Please log in.' })
          return json(res, 200, { ok: true, sessions: store.getProgress(user.id) })
        }
        if (method === 'POST' && pathname === '/api/progress') {
          if (!user) return json(res, 401, { ok: false, message: 'Please log in.' })
          const body = await readBody(req)
          const verifiedByDevice = Boolean(body?.source === 'esp32' && wsBridge.hasRecentTelemetry(user.id, 120_000))
          const saved = await store.addProgress(user.id, { ...body, verifiedByDevice })
          return json(res, 201, { ok: true, session: saved })
        }
        return json(res, 404, { ok: false, message: 'API route not found.' })
      }

      // Redirect legacy /session* paths onto /device*.
      if (pathname === '/session' || pathname.startsWith('/session/')) {
        const suffix = pathname === '/session' ? '/' : pathname.slice('/session'.length)
        res.writeHead(302, { Location: `/device${suffix}` })
        return res.end()
      }

      // Replace the old /device SPA page with the session selector, cycling HUD, and flexion/extension pages.
      if (pathname === '/device' || pathname.startsWith('/device/')) {
        const entryFiles = {
          '/device/': 'session-select.html',
          '/device/cycling/': 'session.html',
          '/device/flexion/': 'flexion.html',
        }
        const trailingRedirects = ['/device', '/device/cycling', '/device/flexion']
        const isEntry = Boolean(entryFiles[pathname]) || trailingRedirects.includes(pathname)

        if (!user) {
          if (isEntry) { res.writeHead(302, { Location: '/' }); return res.end() }
          return json(res, 401, { ok: false, message: 'Please log in.' })
        }

        const deviceMode = resolveDeviceMode(token)
        if (deviceMode === 'none') {
          if (pathname === '/device') return await serveStatic(res, publicDir, 'index.html')
          if (isEntry) { res.writeHead(302, { Location: '/dashboard' }); return res.end() }
          return json(res, 403, { ok: false, message: 'Connect ESP32 or open Demo mode from the dashboard first.' })
        }

        if (trailingRedirects.includes(pathname)) {
          res.writeHead(302, { Location: `${pathname}/` })
          return res.end()
        }

        const entryFile = entryFiles[pathname]
        const relPath = entryFile
          ? entryFile
          : pathname.startsWith('/device/cycling/')
            ? pathname.slice('/device/cycling/'.length)
            : pathname.startsWith('/device/flexion/')
              ? pathname.slice('/device/flexion/'.length)
              : pathname.slice('/device/'.length)
        return await serveStatic(res, softwareDir, relPath)
      }

      // Serve the single-page website (including refreshes of /dashboard, /progress etc.).
      const isPublicAsset = pathname.startsWith('/assets/') || ['/styles.css', '/exercise.css', '/app.js', '/exercise.js'].includes(pathname)
      return await serveStatic(res, publicDir, isPublicAsset ? pathname : 'index.html')
    } catch (e) {
      if (pathname.startsWith('/api/')) return responseError(res, e)
      json(res, 404, { ok: false, message: 'Page not found.' })
    }
  })

  const wsBridge = attachWsBridge(appServer, {
    store,
    resolveDeviceMode,
    getTargetWsUrl: () => runtimeWsUrl,
  })

  return { server: appServer, config, store, wsBridge }
}

const isMain = Boolean(process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url))
const defaultInstance = isMain && process.env.NODE_ENV !== 'test' ? await createAppServer() : null
export const server = defaultInstance?.server ?? null

if (defaultInstance) {
  const { config } = defaultInstance
  defaultInstance.server.listen(config.port, config.host, () => {
    console.log(`\nDOULA website: http://${config.host}:${config.port}`)
    console.log(config.smtp.user && config.smtp.password ? 'OTP delivery: Gmail SMTP configured.' : 'OTP delivery: LOCAL DEMO (code printed in this terminal).')
    console.log(
      config.bypassEsp
        ? 'ESP32: BYPASSED for testing (session routes unlocked).'
        : config.ws
          ? 'ESP32 configured.'
          : 'ESP32: not configured yet (Connect Device will show a clear message; Try without ESP32 opens Demo mode).'
    )
  })
}
