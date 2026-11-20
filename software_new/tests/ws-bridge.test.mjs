import assert from 'node:assert/strict'
import { createHash, randomBytes } from 'node:crypto'
import { mkdtemp, rm } from 'node:fs/promises'
import http from 'node:http'
import path from 'node:path'
import test from 'node:test'
import { fileURLToPath } from 'node:url'
import {
  decodeWsFrames,
  encodeWsFrame,
  isAllowedLanWsUrl,
  normalizeEspWsUrl,
} from '../backend/wsBridge.mjs'
import { createAppServer } from '../server.mjs'

const WS_MAGIC_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

function encodeMaskedClientTextFrame(text) {
  const payload = Buffer.from(text, 'utf8')
  const mask = randomBytes(4)
  const len = payload.length
  const header = len < 126
    ? Buffer.from([0x81, 0x80 | len])
    : Buffer.from([0x81, 0x80 | 126, (len >> 8) & 0xff, len & 0xff])
  const masked = Buffer.allocUnsafe(len)
  for (let i = 0; i < len; i += 1) {
    masked[i] = payload[i] ^ mask[i % 4]
  }
  return Buffer.concat([header, mask, masked])
}

async function createMockEspServer() {
  const server = http.createServer((_req, res) => {
    res.writeHead(404)
    res.end()
  })
  const sockets = new Set()
  const receivedCommands = []

  server.on('upgrade', (req, socket) => {
    const key = req.headers['sec-websocket-key']
    if (!key) return socket.destroy()
    const accept = createHash('sha1').update(`${key}${WS_MAGIC_GUID}`).digest('base64')
    socket.write(
      'HTTP/1.1 101 Switching Protocols\r\n' +
        'Upgrade: websocket\r\n' +
        'Connection: Upgrade\r\n' +
        `Sec-WebSocket-Accept: ${accept}\r\n\r\n`
    )
    sockets.add(socket)
    let buf = Buffer.alloc(0)
    socket.on('data', (chunk) => {
      buf = Buffer.concat([buf, chunk])
      const { frames, remainder } = decodeWsFrames(buf)
      buf = remainder
      for (const frame of frames) {
        if (frame.opcode === 0x8) {
          socket.end(encodeWsFrame(0x8, frame.payload.subarray(0, 2)))
        } else if (frame.opcode === 0x9) {
          socket.write(encodeWsFrame(0xA, frame.payload))
        } else if (frame.opcode === 0x1) {
          receivedCommands.push(JSON.parse(frame.payload.toString('utf8')))
        }
      }
    })
    socket.on('close', () => sockets.delete(socket))
    socket.on('error', () => sockets.delete(socket))
  })

  await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve))
  const { port } = server.address()
  return {
    port,
    wsUrl: `ws://127.0.0.1:${port}/`,
    receivedCommands,
    broadcast(obj) {
      const frame = encodeWsFrame(0x1, Buffer.from(JSON.stringify(obj), 'utf8'))
      for (const s of sockets) {
        if (!s.destroyed) s.write(frame)
      }
    },
    async close() {
      for (const s of sockets) { try { s.destroy() } catch {} }
      sockets.clear()
      server.closeAllConnections?.()
      await new Promise((resolve) => server.close(resolve))
    },
  }
}

async function openRawWsClient(port, pathName, cookieHeader) {
  return await new Promise((resolve, reject) => {
    const key = randomBytes(16).toString('base64')
    const req = http.request({
      hostname: '127.0.0.1',
      port,
      path: pathName,
      method: 'GET',
      headers: {
        Connection: 'Upgrade',
        Upgrade: 'websocket',
        'Sec-WebSocket-Key': key,
        'Sec-WebSocket-Version': '13',
        ...(cookieHeader ? { Cookie: cookieHeader } : {}),
      },
    })
    req.on('response', (res) => {
      res.destroy()
      req.destroy()
      reject(new Error(`Unexpected HTTP status ${res.statusCode}`))
    })
    req.on('upgrade', (_res, socket, head) => {
      const messages = []
      let buf = head ? Buffer.from(head) : Buffer.alloc(0)
      const listeners = []

      const pump = () => {
        const { frames, remainder } = decodeWsFrames(buf)
        buf = remainder
        for (const f of frames) {
          if (f.opcode === 0x1) {
            const parsed = JSON.parse(f.payload.toString('utf8'))
            messages.push(parsed)
            while (listeners.length) {
              const fn = listeners.shift()
              fn()
            }
          }
        }
      }
      pump()
      socket.on('data', (chunk) => {
        buf = Buffer.concat([buf, chunk])
        pump()
      })
      resolve({
        socket,
        messages,
        async waitForMessage(predicate, timeoutMs = 3000) {
          const hit = messages.find(predicate)
          if (hit) return hit
          return await new Promise((resWait, rejWait) => {
            const timer = setTimeout(() => rejWait(new Error('Timed out waiting for WS message')), timeoutMs)
            const check = () => {
              const found = messages.find(predicate)
              if (found) {
                clearTimeout(timer)
                resWait(found)
              } else {
                listeners.push(check)
              }
            }
            listeners.push(check)
          })
        },
        sendJson(obj) {
          socket.write(encodeMaskedClientTextFrame(JSON.stringify(obj)))
        },
        close() {
          try { socket.destroy() } catch {}
        },
      })
    })
    req.on('error', reject)
    req.end()
  })
}

async function signupSession(baseUrl, email, otpCodes) {
  const reqOtp = await fetch(`${baseUrl}/api/auth/start`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name: 'Bench Tester', email }),
  })
  assert.equal(reqOtp.status, 200)
  const verify = await fetch(`${baseUrl}/api/auth/verify`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email, code: otpCodes[email] }),
  })
  assert.equal(verify.status, 200)
  const verifiedCookie = verify.headers.get('set-cookie').split(';')[0]
  const finish = await fetch(`${baseUrl}/api/auth/finish`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', Cookie: verifiedCookie },
    body: JSON.stringify({ password: 'TestPassword123!' }),
  })
  assert.equal(finish.status, 200)
  const sessionCookie = finish.headers.getSetCookie().find((c) => c.startsWith('doula_session='))
  assert.ok(sessionCookie)
  return sessionCookie.split(';')[0]
}

test('LAN URL validation & RFC6455 frame codec work cleanly', () => {
  assert.equal(normalizeEspWsUrl('192.168.1.42'), 'ws://192.168.1.42:81/')
  assert.equal(normalizeEspWsUrl('doula-esp32.local'), 'ws://doula-esp32.local:81/')
  assert.equal(normalizeEspWsUrl('ws://10.0.0.9:81/'), 'ws://10.0.0.9:81/')
  assert.equal(isAllowedLanWsUrl('ws://192.168.1.42:81/'), true)
  assert.equal(isAllowedLanWsUrl('ws://172.16.0.5:81/'), true)
  assert.equal(isAllowedLanWsUrl('ws://doula-esp32.local:81/'), true)
  assert.equal(isAllowedLanWsUrl('ws://8.8.8.8:81/'), false)
  assert.equal(isAllowedLanWsUrl('ws://172.32.0.5:81/'), false)
  assert.equal(isAllowedLanWsUrl('ws://example.com:81/'), false)

  const longPayload = 'x'.repeat(240)
  const frame = encodeWsFrame(0x1, Buffer.from(longPayload, 'utf8'))
  const { frames, remainder } = decodeWsFrames(frame)
  assert.equal(frames.length, 1)
  assert.equal(frames[0].payload.toString('utf8'), longPayload)
  assert.equal(remainder.length, 0)
})

test('Live /ws/device bridge relays commands, telemetry, torque updates, and ESP32 verified logs', async (t) => {
  const testsDir = path.dirname(fileURLToPath(import.meta.url))
  const dataDir = await mkdtemp(path.join(testsDir, '.ws-test-data-'))
  const otpCodes = {}
  const origLog = console.log
  console.log = (...args) => {
    const line = args.join(' ')
    const match = /OTP for ([^\s:]+):\s*(\d{4})/i.exec(line)
    if (match) otpCodes[match[1].toLowerCase()] = match[2]
  }

  const mockEsp = await createMockEspServer()
  const { server } = await createAppServer({
    host: '127.0.0.1',
    port: 0,
    dataDir,
    smtp: { user: '', password: '' },
    wsUrl: '',
    bypassEsp: false,
  })
  await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve))
  const appPort = server.address().port
  const baseUrl = `http://127.0.0.1:${appPort}`

  t.after(async () => {
    console.log = origLog
    await mockEsp.close()
    server.closeAllConnections?.()
    await new Promise((resolve) => server.close(resolve))
    await rm(dataDir, { recursive: true, force: true })
  })

  const cookie = await signupSession(baseUrl, 'esplive@gmail.com', otpCodes)
  const otherCookie = await signupSession(baseUrl, 'otheruser@gmail.com', otpCodes)

  // Reject unauthenticated and demo-mode bridge upgrades
  await assert.rejects(() => openRawWsClient(appPort, '/ws/device', ''), /401/)
  await fetch(`${baseUrl}/api/device/demo`, { method: 'POST', headers: { Cookie: cookie } })
  await assert.rejects(() => openRawWsClient(appPort, '/ws/device', cookie), /403/)

  // Reject public non-LAN IP in /api/device/connect
  const badConnect = await fetch(`${baseUrl}/api/device/connect`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', Cookie: cookie },
    body: JSON.stringify({ wsUrl: 'ws://8.8.8.8:81/' }),
  })
  assert.equal(badConnect.status, 400)

  // Connect runtime ESP32 IP dynamically from Dashboard API
  const goodConnect = await fetch(`${baseUrl}/api/device/connect`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', Cookie: cookie },
    body: JSON.stringify({ wsUrl: `127.0.0.1:${mockEsp.port}` }),
  })
  const goodData = await goodConnect.json()
  assert.equal(goodConnect.status, 200)
  assert.equal(goodData.connected, true)
  assert.equal(goodData.wsUrl, `ws://127.0.0.1:${mockEsp.port}/`)

  const cfgRes = await fetch(`${baseUrl}/api/device/config`, { headers: { Cookie: cookie } })
  const cfg = await cfgRes.json()
  assert.equal(cfg.deviceMode, 'connected')
  assert.equal(cfg.bridgeWsPath, '/ws/device')
  assert.equal(cfg.wsUrl, `ws://127.0.0.1:${mockEsp.port}/`)

  // Verify Cycling page exposes +/- torque bar and bridge wiring
  const cycHtml = await (await fetch(`${baseUrl}/device/cycling/`, { headers: { Cookie: cookie } })).text()
  assert.match(cycHtml, /cyclingTorqueMinusBtn/)
  assert.match(cycHtml, /cyclingTorquePlusBtn/)

  // Open browser -> Node bridge WebSocket
  const client = await openRawWsClient(appPort, '/ws/device', cookie)
  t.after(() => client.close())

  const statusMsg = await client.waitForMessage((m) => m.type === 'esp_status' && m.connected === true)
  assert.equal(statusMsg.connected, true)

  // Browser sends start_session & + / - torque adjustments -> ESP receives both
  client.sendJson({ type: 'start_session', mode: 'cycling', op_mode: 7, torque_nm: 14, target_torque_ext: 14 })
  client.sendJson({ type: 'set_torque', mode: 'cycling', op_mode: 7, torque_nm: 18, target_torque_ext: 18, delta: 4 })

  const deadline = Date.now() + 2000
  while (mockEsp.receivedCommands.length < 2 && Date.now() < deadline) {
    await new Promise((r) => setTimeout(r, 20))
  }
  assert.equal(mockEsp.receivedCommands.length, 2)
  assert.equal(mockEsp.receivedCommands[0].type, 'start_session')
  assert.equal(mockEsp.receivedCommands[1].type, 'set_torque')
  assert.equal(mockEsp.receivedCommands[1].torque_nm, 18)

  // ESP streams telemetry -> browser receives it cleanly
  mockEsp.broadcast({ type: 'telemetry', mode: 'cycling', rpm: 74.2, speed: 23.0, torque: 18.0, reps: 4 })
  const telemMsg = await client.waitForMessage((m) => m.type === 'telemetry')
  assert.equal(telemMsg.rpm, 74.2)
  assert.equal(telemMsg.torque, 18.0)
  assert.equal(telemMsg.reps, 4)

  // Flexion 120-180 deg setup, begin_exercise, park_home, and spasm telemetry relay cleanly
  client.sendJson({ type: 'start_session', mode: 'passive', op_mode: 3, role: 'support', rom_min_deg: 120, rom_max_deg: 180, torque_nm: 12, target_vel_deg_s: 18 })
  client.sendJson({ type: 'begin_exercise' })
  client.sendJson({ type: 'park_home' })

  const flexDeadline = Date.now() + 2000
  while (mockEsp.receivedCommands.length < 5 && Date.now() < flexDeadline) {
    await new Promise((r) => setTimeout(r, 20))
  }
  assert.equal(mockEsp.receivedCommands.length, 5)
  assert.equal(mockEsp.receivedCommands[2].type, 'start_session')
  assert.equal(mockEsp.receivedCommands[2].rom_min_deg, 120)
  assert.equal(mockEsp.receivedCommands[2].rom_max_deg, 180)
  assert.equal(mockEsp.receivedCommands[3].type, 'begin_exercise')
  assert.equal(mockEsp.receivedCommands[4].type, 'park_home')

  mockEsp.broadcast({ type: 'telemetry', mode: 'passive', setup_state: 'moving_to_start', setup_warning: 1, positioning: 1, knee_angle_deg: 45.0, spasm: 0 })
  const setupMsg = await client.waitForMessage((m) => m.type === 'telemetry' && m.setup_state === 'moving_to_start')
  assert.equal(setupMsg.setup_warning, 1)
  assert.equal(setupMsg.positioning, 1)
  assert.equal(setupMsg.knee_angle_deg, 45.0)

  mockEsp.broadcast({ type: 'telemetry', mode: 'passive', spasm: 1, paused: 1 })
  const spasmMsg = await client.waitForMessage((m) => m.type === 'telemetry' && m.spasm === 1)
  assert.equal(spasmMsg.paused, 1)

  // Saving esp32 progress for verified user sets verifiedByDevice=true; unverified user gets false
  const saveOk = await fetch(`${baseUrl}/api/progress`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', Cookie: cookie },
    body: JSON.stringify({ type: 'Cycling', mode: 'Cycling', torqueNm: 18, reps: 4, durationSeconds: 18, source: 'esp32' }),
  })
  const saveOkBody = await saveOk.json()
  assert.equal(saveOk.status, 201)
  assert.equal(saveOkBody.session.source, 'esp32')
  assert.equal(saveOkBody.session.verifiedByDevice, true)

  const saveSpoof = await fetch(`${baseUrl}/api/progress`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', Cookie: otherCookie },
    body: JSON.stringify({ type: 'Cycling', mode: 'Cycling', torqueNm: 18, reps: 4, durationSeconds: 18, source: 'esp32' }),
  })
  const saveSpoofBody = await saveSpoof.json()
  assert.equal(saveSpoof.status, 201)
  assert.equal(saveSpoofBody.session.verifiedByDevice, false)
})
