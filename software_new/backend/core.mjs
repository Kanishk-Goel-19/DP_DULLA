import { randomBytes, randomInt, scrypt as scryptCallback, timingSafeEqual, createHash } from 'node:crypto'
import { promisify } from 'node:util'
import { readFile, writeFile, mkdir, rename } from 'node:fs/promises'
import path from 'node:path'

const scrypt = promisify(scryptCallback)
const gmail = /^[^\s@]+@gmail\.com$/i
const normalizeEmail = value => String(value ?? '').trim().toLowerCase()
const digest = text => createHash('sha256').update(text).digest('hex')

export class AuthStore {
  constructor(dataDirectory) {
    this.directory = dataDirectory
    this.usersFile = path.join(dataDirectory, 'users.json')
    this.progressFile = path.join(dataDirectory, 'progress.json')
    this.users = []
    this.signups = new Map()
    this.verified = new Map()
    this.sessions = new Map()
  }

  async init() {
    await mkdir(this.directory, { recursive: true })
    try { this.users = JSON.parse(await readFile(this.usersFile, 'utf8')) }
    catch (e) { if (e.code !== 'ENOENT') throw e; this.users = [] }
    try { this.progress = JSON.parse(await readFile(this.progressFile, 'utf8')) }
    catch (e) { if (e.code !== 'ENOENT') throw e; this.progress = [] }
  }

  async saveUsers() {
    const temp = this.usersFile + '.tmp'
    await writeFile(temp, JSON.stringify(this.users, null, 2))
    await rename(temp, this.usersFile)
  }

  get(email) { return this.users.find(u => u.email === normalizeEmail(email)) }

  startSignup(name, email) {
    name = String(name ?? '').trim().slice(0, 100)
    email = normalizeEmail(email)
    if (!name) throw new Error('Please enter your name.')
    if (!gmail.test(email)) throw new Error('Enter a Gmail address ending in @gmail.com.')
    if (this.get(email)) throw new Error('An account already exists with this Gmail.')
    const code = String(randomInt(0, 10000)).padStart(4, '0')
    this.signups.set(email, { name, digest: digest(code) })
    return { email, code }
  }

  verifySignup(email, code) {
    email = normalizeEmail(email)
    const current = this.signups.get(email)
    if (!current || !/^\d{4}$/.test(String(code)) || current.digest !== digest(String(code))) {
      throw new Error('Incorrect OTP. Please check your four-digit code.')
    }
    this.signups.delete(email)
    const ticket = randomBytes(24).toString('hex')
    this.verified.set(ticket, { email, name: current.name })
    return ticket
  }

  async finishSignup(ticket, password) {
    const current = this.verified.get(ticket)
    if (!current) throw new Error('Please verify your Gmail first.')
    if (typeof password !== 'string' || password.length < 6) throw new Error('Password must have at least 6 characters.')
    if (this.get(current.email)) throw new Error('An account already exists with this Gmail.')
    const user = { id: randomBytes(12).toString('hex'), name: current.name, email: current.email, passwordHash: await hashPassword(password) }
    this.users.push(user)
    await this.saveUsers()
    this.verified.delete(ticket)
    return this.newSession(user)
  }

  async login(email, password) {
    const user = this.get(email)
    if (!user || !(await checkPassword(String(password ?? ''), user.passwordHash))) {
      throw new Error('Incorrect Gmail or password.')
    }
    return this.newSession(user)
  }

  newSession(user) {
    const token = randomBytes(32).toString('hex')
    this.sessions.set(token, user.id)
    return { token, user: this.profile(user) }
  }

  profile(user) { return { id: user.id, name: user.name, email: user.email } }

  getSession(token) {
    const id = this.sessions.get(token)
    const user = id && this.users.find(u => u.id === id)
    return user ? this.profile(user) : null
  }

  logout(token) { this.sessions.delete(token) }
  getProgress(userId) { return this.progress.filter(row => row.userId === userId) }
}

export async function hashPassword(password) {
  const salt = randomBytes(16).toString('hex')
  const key = await scrypt(password, salt, 64)
  return salt + ':' + key.toString('hex')
}

export async function checkPassword(password, hash) {
  const [salt, expectedHex] = String(hash || '').split(':')
  if (!salt || !expectedHex) return false
  const actual = await scrypt(password, salt, 64)
  const expected = Buffer.from(expectedHex, 'hex')
  return expected.length === actual.length && timingSafeEqual(actual, expected)
}
