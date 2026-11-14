import test from 'node:test'
import assert from 'node:assert/strict'
import { mkdtemp, rm } from 'node:fs/promises'
import { tmpdir } from 'node:os'
import path from 'node:path'
import { AuthStore, checkPassword, hashPassword } from '../backend/core.mjs'

test('passwords are hashed and validated', async () => {
  const stored = await hashPassword('password123')
  assert.ok(!stored.includes('password123'))
  assert.equal(await checkPassword('password123', stored), true)
  assert.equal(await checkPassword('incorrect', stored), false)
})

test('4-digit signup -> verify -> password -> login -> logout', async () => {
  const directory = await mkdtemp(path.join(tmpdir(), 'doula-'))
  try {
    const store = new AuthStore(directory)
    await store.init()
    const { email, code } = store.startSignup('Priyanshi', 'Example@gmail.com')
    assert.equal(email, 'example@gmail.com')
    // Signup only accepts Gmail, not arbitrary email providers.
    assert.equal(code.length, 4)
    assert.match(code, /^\d{4}$/)
    assert.throws(() => store.verifySignup(email, 'not four digits'))
    const ticket = store.verifySignup(email, code)
    const { token, user } = await store.finishSignup(ticket, 'secret123')
    assert.equal(user.name, 'Priyanshi')
    assert.equal(store.getSession(token)?.email, email)
    assert.equal(store.users[0].passwordHash.includes('secret123'), false)
    store.logout(token)
    assert.equal(store.getSession(token), null)
    await assert.rejects(() => store.login(email, 'wrong'))
    const loggedIn = await store.login(email, 'secret123')
    assert.equal(loggedIn.user.email, email)
    const restarted = new AuthStore(directory)
    await restarted.init()
    assert.equal((await restarted.login(email, 'secret123')).user.email, email)
  } finally {
    await rm(directory, { recursive: true, force: true })
  }
})
