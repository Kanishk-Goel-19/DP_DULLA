import { createHash } from 'node:crypto'

const WS_MAGIC_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

export function parseCookieHeader(cookieHeader = '') {
  return Object.fromEntries(
    String(cookieHeader)
      .split(';')
      .map((c) => c.trim())
      .filter(Boolean)
      .map((c) => {
        const pos = c.indexOf('=')
        return pos === -1 ? [c, ''] : [c.slice(0, pos), decodeURIComponent(c.slice(pos + 1))]
      })
  )
}

export function normalizeEspWsUrl(rawInput) {
  const raw = String(rawInput ?? '').trim()
  if (!raw) throw new Error('Enter an ESP32 IP or WebSocket URL.')
  const withScheme = /^wss?:\/\//i.test(raw) ? raw : `ws://${raw}`
  let parsed
  try {
    parsed = new URL(withScheme)
  } catch {
    throw new Error('Invalid ESP32 address. Example: ws://192.168.1.42:81/ or 192.168.1.42')
  }
  if (!['ws:', 'wss:'].includes(parsed.protocol)) {
    throw new Error('ESP32 URL must use ws:// or wss://.')
  }
  if (!parsed.port && !/^wss?:\/\//i.test(raw)) {
    parsed.port = '81'
  }
  return parsed.toString()
}

export function isAllowedLanWsUrl(urlString) {
  let parsed
  try {
    parsed = new URL(String(urlString || ''))
  } catch {
    return false
  }
  if (!['ws:', 'wss:'].includes(parsed.protocol)) return false
  const host = parsed.hostname.replace(/^\[|\]$/g, '').toLowerCase()
  if (!host) return false
  if (host === 'localhost' || host === '127.0.0.1' || host === '::1' || host.endsWith('.local')) {
    return true
  }
  const ipv4 = /^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$/.exec(host)
  if (!ipv4) return false
  const octets = ipv4.slice(1).map(Number)
  if (octets.some((n) => n < 0 || n > 255)) return false
  const [a, b] = octets
  return a === 10 || (a === 192 && b === 168) || (a === 172 && b >= 16 && b <= 31) || (a === 169 && b === 254)
}

export function encodeWsFrame(opcode, payloadBuffer) {
  const payload = Buffer.isBuffer(payloadBuffer) ? payloadBuffer : Buffer.from(String(payloadBuffer), 'utf8')
  const len = payload.length
  let header
  if (len < 126) {
    header = Buffer.allocUnsafe(2)
    header[0] = 0x80 | (opcode & 0x0f)
    header[1] = len
  } else if (len < 65536) {
    header = Buffer.allocUnsafe(4)
    header[0] = 0x80 | (opcode & 0x0f)
    header[1] = 126
    header.writeUInt16BE(len, 2)
  } else {
    header = Buffer.allocUnsafe(10)
    header[0] = 0x80 | (opcode & 0x0f)
    header[1] = 127
    header.writeBigUInt64BE(BigInt(len), 2)
  }
  return Buffer.concat([header, payload])
}

export function decodeWsFrames(buffer) {
  const frames = []
  let offset = 0

  while (offset + 2 <= buffer.length) {
    const first = buffer[offset]
    const second = buffer[offset + 1]
    const opcode = first & 0x0f
    const masked = (second & 0x80) !== 0
    let len = second & 0x7f
    let cursor = offset + 2

    if (len === 126) {
      if (cursor + 2 > buffer.length) break
      len = buffer.readUInt16BE(cursor)
      cursor += 2
    } else if (len === 127) {
      if (cursor + 8 > buffer.length) break
      const bigLen = buffer.readBigUInt64BE(cursor)
      if (bigLen > 1_048_576n) throw new Error('WebSocket frame too large')
      len = Number(bigLen)
      cursor += 8
    }

    const maskOffset = cursor
    if (masked) {
      if (cursor + 4 > buffer.length) break
      cursor += 4
    }

    if (cursor + len > buffer.length) break
    const payload = Buffer.allocUnsafe(len)
    buffer.copy(payload, 0, cursor, cursor + len)
    if (masked) {
      for (let i = 0; i < len; i += 1) {
        payload[i] ^= buffer[maskOffset + (i % 4)]
      }
    }

    frames.push({ fin: (first & 0x80) !== 0, opcode, payload })
    offset = cursor + len
  }

  return { frames, remainder: buffer.subarray(offset) }
}

function rejectUpgrade(socket, statusLine, message) {
  try {
    socket.write(`HTTP/1.1 ${statusLine}\r\nConnection: close\r\nContent-Type: text/plain\r\n\r\n${message}`)
  } catch {}
  socket.destroy()
}

export function attachWsBridge(appServer, { store, resolveDeviceMode, getTargetWsUrl }) {
  const clients = new Map()
  const lastHardwarePacketAt = new Map()

  let upstream = null
  let upstreamUrl = ''
  let upstreamConnected = false
  let reconnectTimer = null
  let idleCloseTimer = null
  let closed = false

  function sendJson(socket, value) {
    if (!socket || socket.destroyed) return
    try {
      socket.write(encodeWsFrame(0x1, Buffer.from(JSON.stringify(value), 'utf8')))
    } catch {}
  }

  function broadcastText(text) {
    const frame = encodeWsFrame(0x1, Buffer.from(text, 'utf8'))
    for (const [sock, meta] of clients) {
      if (sock.destroyed) {
        clients.delete(sock)
        continue
      }
      lastHardwarePacketAt.set(meta.userId, Date.now())
      try {
        sock.write(frame)
      } catch {}
    }
  }

  function broadcastStatus(message) {
    const msg = JSON.stringify({
      type: 'esp_status',
      connected: upstreamConnected,
      url: upstreamUrl,
      message,
    })
    const frame = encodeWsFrame(0x1, Buffer.from(msg, 'utf8'))
    for (const sock of clients.keys()) {
      if (!sock.destroyed) {
        try { sock.write(frame) } catch {}
      }
    }
  }

  function clearTimers() {
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null }
    if (idleCloseTimer) { clearTimeout(idleCloseTimer); idleCloseTimer = null }
  }

  function closeUpstream() {
    if (upstream) {
      try { upstream.close() } catch {}
      upstream = null
    }
    upstreamConnected = false
  }

  function scheduleReconnect() {
    if (closed || clients.size === 0 || reconnectTimer) return
    const nextUrl = getTargetWsUrl()
    if (!nextUrl || !/^wss?:\/\//i.test(nextUrl)) return
    reconnectTimer = setTimeout(() => {
      reconnectTimer = null
      ensureUpstream()
    }, 2000)
    reconnectTimer.unref?.()
  }

  function ensureUpstream() {
    if (closed || clients.size === 0) return
    if (idleCloseTimer) {
      clearTimeout(idleCloseTimer)
      idleCloseTimer = null
    }
    const nextUrl = getTargetWsUrl()
    if (!nextUrl || !/^wss?:\/\//i.test(nextUrl)) {
      closeUpstream()
      upstreamUrl = ''
      broadcastStatus('No ESP32 WebSocket URL configured yet.')
      return
    }
    if (upstream && upstreamUrl === nextUrl && (upstream.readyState === WebSocket.OPEN || upstream.readyState === WebSocket.CONNECTING)) {
      return
    }

    closeUpstream()
    upstreamUrl = nextUrl
    try {
      const sock = new WebSocket(nextUrl)
      upstream = sock
      sock.addEventListener('open', () => {
        if (upstream !== sock || closed) return
        upstreamConnected = true
        broadcastStatus('Connected to ESP32 telemetry stream.')
      })
      sock.addEventListener('message', (event) => {
        if (upstream !== sock || closed) return
        const payload = typeof event.data === 'string' ? event.data : String(event.data || '')
        broadcastText(payload)
      })
      sock.addEventListener('close', () => {
        if (upstream !== sock) return
        upstream = null
        upstreamConnected = false
        broadcastStatus('ESP32 disconnected — retrying...')
        scheduleReconnect()
      })
      sock.addEventListener('error', () => {
        if (upstream !== sock) return
        upstreamConnected = false
        broadcastStatus('Unable to reach ESP32 — retrying...')
      })
    } catch {
      upstream = null
      upstreamConnected = false
      scheduleReconnect()
    }
  }

  function handleUpgrade(req, socket) {
    if (closed) return socket.destroy()
    const pathname = new URL(req.url || '/', `http://${req.headers.host || 'localhost'}`).pathname
    if (pathname !== '/ws/device') {
      return rejectUpgrade(socket, '404 Not Found', 'Unknown WebSocket endpoint.')
    }

    const cookies = parseCookieHeader(req.headers.cookie)
    const token = cookies.doula_session
    const user = token ? store.getSession(token) : null
    if (!user) {
      return rejectUpgrade(socket, '401 Unauthorized', 'Please log in first.')
    }
    if (resolveDeviceMode(token) !== 'connected') {
      return rejectUpgrade(socket, '403 Forbidden', 'Device bridge requires connected ESP32 mode.')
    }

    const wsKey = req.headers['sec-websocket-key']
    if (!wsKey || Array.isArray(wsKey)) {
      return rejectUpgrade(socket, '400 Bad Request', 'Missing WebSocket key.')
    }

    const accept = createHash('sha1').update(`${wsKey}${WS_MAGIC_GUID}`).digest('base64')
    socket.write(
      'HTTP/1.1 101 Switching Protocols\r\n' +
        'Upgrade: websocket\r\n' +
        'Connection: Upgrade\r\n' +
        `Sec-WebSocket-Accept: ${accept}\r\n\r\n`
    )

    clients.set(socket, { userId: user.id, token })
    let buffer = Buffer.alloc(0)

    ensureUpstream()
    sendJson(socket, {
      type: 'esp_status',
      connected: upstreamConnected,
      url: getTargetWsUrl(),
      message: upstreamConnected ? 'Connected to ESP32 telemetry stream.' : 'Connecting to ESP32 telemetry stream...',
    })

    socket.on('data', (chunk) => {
      buffer = Buffer.concat([buffer, chunk])
      let decoded
      try {
        decoded = decodeWsFrames(buffer)
      } catch {
        socket.destroy()
        return
      }
      buffer = decoded.remainder

      for (const frame of decoded.frames) {
        if (frame.opcode === 0x8) {
          try { socket.write(encodeWsFrame(0x8, frame.payload.subarray(0, 2))) } catch {}
          socket.end()
          return
        }
        if (frame.opcode === 0x9) {
          try { socket.write(encodeWsFrame(0xA, frame.payload)) } catch {}
          continue
        }
        if (frame.opcode === 0x1) {
          const text = frame.payload.toString('utf8')
          if (upstream && upstream.readyState === WebSocket.OPEN) {
            upstream.send(text)
          } else {
            ensureUpstream()
            sendJson(socket, {
              type: 'esp_ack',
              forwarded: false,
              message: 'ESP32 is not connected yet — reconnect after powering on hardware.',
            })
          }
        }
      }
    })

    const cleanup = () => {
      clients.delete(socket)
      if (clients.size === 0 && !closed && !idleCloseTimer) {
        idleCloseTimer = setTimeout(() => {
          idleCloseTimer = null
          if (clients.size === 0) closeUpstream()
        }, 3000)
        idleCloseTimer.unref?.()
      }
    }
    socket.on('close', cleanup)
    socket.on('error', cleanup)
  }

  function shutdown() {
    if (closed) return
    closed = true
    clearTimers()
    closeUpstream()
    for (const sock of clients.keys()) {
      try { sock.destroy() } catch {}
    }
    clients.clear()
  }

  appServer.on('upgrade', handleUpgrade)
  appServer.on('close', shutdown)
  const originalServerClose = appServer.close.bind(appServer)
  appServer.close = (cb) => {
    shutdown()
    appServer.closeAllConnections?.()
    return originalServerClose(cb)
  }

  return {
    resetUpstream() {
      clearTimers()
      closeUpstream()
      ensureUpstream()
    },
    close: shutdown,
    hasRecentTelemetry(userId, withinMs = 15000) {
      const ts = lastHardwarePacketAt.get(userId)
      return Boolean(ts && Date.now() - ts <= withinMs)
    },
    getUpstreamStatus() {
      return { connected: upstreamConnected, url: getTargetWsUrl() }
    },
  }
}
