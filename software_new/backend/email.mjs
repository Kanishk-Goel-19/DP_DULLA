import tls from 'node:tls'
import readline from 'node:readline'

// Uses the Gmail SMTP server directly, without requiring any npm packages.
// The sender must use a Google App Password, not their Gmail login password.
export async function sendVerificationEmail(to, code, config) {
  if (!config.user || !config.password) {
    console.log(`\n[DOULA LOCAL DEMO] 4-digit OTP for ${to}: ${code}\n`)
    return 'terminal'
  }

  const socket = tls.connect({ host: 'smtp.gmail.com', port: 465, servername: 'smtp.gmail.com', rejectUnauthorized: true })
  socket.setTimeout(12000, () => socket.destroy(new Error('Gmail connection timed out')))
  const input = readline.createInterface({ input: socket, crlfDelay: Infinity })
  const lines = []
  const readers = []
  let socketError = null

  input.on('line', line => {
    const resolve = readers.shift()
    if (resolve) resolve(line)
    else lines.push(line)
  })
  socket.on('error', e => {
    socketError = e
    for (const resolve of readers.splice(0)) resolve(null)
  })
  socket.on('close', () => {
    socketError ??= new Error('SMTP connection closed unexpectedly')
    for (const resolve of readers.splice(0)) resolve(null)
  })

  const nextLine = () => lines.length ? Promise.resolve(lines.shift()) : new Promise(resolve => readers.push(resolve))
  async function response(expected) {
    let message = ''
    let status
    for (;;) {
      const line = await nextLine()
      if (line === null) throw socketError ?? new Error('SMTP connection closed')
      message += line + '\n'
      const match = /^(\d{3})([ -])/.exec(line)
      if (match?.[2] === ' ') { status = Number(match[1]); break }
    }
    if (status !== expected) throw new Error(`SMTP response ${status}: ${message.trim()}`)
  }
  async function command(text, expected) {
    socket.write(text + '\r\n')
    await response(expected)
  }

  try {
    await response(220)
    await command('EHLO doula.local', 250)
    await command('AUTH LOGIN', 334)
    await command(Buffer.from(config.user).toString('base64'), 334)
    await command(Buffer.from(config.password).toString('base64'), 235)
    await command(`MAIL FROM:<${config.user}>`, 250)
    await command(`RCPT TO:<${to}>`, 250)
    await command('DATA', 354)
    const message = [
      `From: "DOULA" <${config.user}>`,
      `To: <${to}>`,
      'Subject: Your DOULA verification code',
      'MIME-Version: 1.0',
      'Content-Type: text/plain; charset=UTF-8',
      '',
      `Your DOULA account verification code is: ${code}`,
      '',
      'Enter this four-digit code on the DOULA signup screen.',
      ''
    ].join('\r\n')
    socket.write(message.replace(/\r\n\./g, '\r\n..') + '\r\n.\r\n')
    await response(250)
    await command('QUIT', 221)
    return 'email'
  } finally {
    input.close()
    socket.destroy()
  }
}
