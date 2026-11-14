import test from 'node:test'
import assert from 'node:assert/strict'
import { mkdtemp, rm } from 'node:fs/promises'
import { tmpdir } from 'node:os'
import path from 'node:path'
import { AuthStore } from '../backend/core.mjs'

test('exercise sessions are saved, separated by user and survive restart', async () => {
  const directory = await mkdtemp(path.join(tmpdir(), 'doula-progress-'))
  try {
    const store = new AuthStore(directory)
    await store.init()
    for (const [name, email] of [['Priyanshi', 'priyanshi@gmail.com'], ['Teammate', 'teammate@gmail.com']]) {
      const signup = store.startSignup(name, email)
      const ticket = store.verifySignup(signup.email, signup.code)
      await store.finishSignup(ticket, 'mypassword')
    }
    const first = store.get('priyanshi@gmail.com')
    const second = store.get('teammate@gmail.com')
    const saved = await store.addProgress(first.id, { type: 'Support', reps: 10, source: 'demo', durationSeconds: 36 })
    assert.equal(saved.type, 'Support')
    assert.equal(saved.reps, 10)
    assert.equal(saved.source, 'demo')
    assert.equal(saved.verifiedByDevice, false)
    assert.ok(!('userId' in saved))
    assert.equal(store.getProgress(second.id).length, 0)
    await store.addProgress(second.id, { type: 'Cycling', reps: 5, source: 'connected-preview', durationSeconds: 18 })
    assert.equal(store.getProgress(first.id).length, 1)
    assert.equal(store.getProgress(second.id).length, 1)
    await assert.rejects(store.addProgress(first.id, { type: 'Unknown', reps: 10, source: 'demo' }))
    await assert.rejects(store.addProgress(first.id, { type: 'Support', reps: 100, source: 'demo' }))
    await assert.rejects(store.addProgress('unknown-user', { type: 'Support', reps: 1, source: 'demo' }))

    const restarted = new AuthStore(directory)
    await restarted.init()
    assert.equal(restarted.getProgress(first.id)[0].id, saved.id)
    assert.equal(restarted.getProgress(second.id)[0].type, 'Cycling')
  } finally {
    await rm(directory, { recursive: true, force: true })
  }
})
