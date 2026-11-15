import test from 'node:test'
import assert from 'node:assert/strict'
import { mkdtemp, rm } from 'node:fs/promises'
import { tmpdir } from 'node:os'
import path from 'node:path'
import { computeClinicalPrescription, normalizeIntake } from '../backend/clinical.mjs'
import { AuthStore } from '../backend/core.mjs'
import { createAppServer } from '../server.mjs'

test('Section 3.2 Clinical Decision Tree routes all acute, early ROM, and intermediate regimes', () => {
  // Gate 1: Acute Protection -> REC_CPM (VAS >= 6)
  const gate1Vas = computeClinicalPrescription({
    pathology: 'ACLR',
    weeksPostOp: 3,
    painVas: 7,
    effusion: 'moderate',
    romExt: 5,
    romFlex: 75,
    bodyMassKg: 72,
  })
  assert.equal(gate1Vas.protocolId, 'REC_CPM')
  assert.equal(gate1Vas.recommendedSession, 'flexion')
  assert.equal(gate1Vas.flexion.modeId, 'passive')
  assert.equal(gate1Vas.cycling.allowed, false)

  // Gate 1: Weeks < 1 or severe effusion
  const gate1Early = computeClinicalPrescription({
    pathology: 'TKA',
    weeksPostOp: 0.5,
    painVas: 3,
    effusion: 'trace',
    romExt: 5,
    romFlex: 110,
  })
  assert.equal(gate1Early.protocolId, 'REC_CPM')
  assert.equal(gate1Early.cycling.allowed, false)

  // Gate 2 (ROM < 100) + Gate 3 YES (Weeks < 3 or VAS >= 4) -> REC_ISO_CPM
  const gate3Yes = computeClinicalPrescription({
    pathology: 'ACLR',
    weeksPostOp: 2,
    painVas: 3,
    effusion: 'trace',
    romExt: 0,
    romFlex: 85,
  })
  assert.equal(gate3Yes.protocolId, 'REC_ISO_CPM')
  assert.equal(gate3Yes.flexion.modeId, 'passive')
  assert.equal(gate3Yes.flexion.suggestedReps, 10)
  assert.equal(gate3Yes.cycling.allowed, false)

  // Gate 2 (ROM < 100) + Gate 3 NO (Weeks >= 3 & VAS <= 3) -> REC_EARLY_ISO
  const gate3No = computeClinicalPrescription({
    pathology: 'ACLR',
    weeksPostOp: 4,
    painVas: 2,
    effusion: 'trace',
    romExt: 0,
    romFlex: 90,
  })
  assert.equal(gate3No.protocolId, 'REC_EARLY_ISO')
  assert.equal(gate3No.recommendedSession, 'flexion')
  assert.equal(gate3No.flexion.modeId, 'isotonic')
  assert.equal(gate3No.flexion.targetTorqueNm, 20)
  assert.equal(gate3No.cycling.allowed, false)

  // Gate 2 (ROM >= 100) + Gate 4 Early (Weeks 1-5) + ACLR -> REC_CYC_EARLY prioritizing cycling
  const earlyAclCycling = computeClinicalPrescription({
    pathology: 'ACLR',
    weeksPostOp: 4,
    painVas: 2,
    effusion: 'none',
    romExt: 0,
    romFlex: 110,
  })
  assert.equal(earlyAclCycling.protocolId, 'REC_CYC_EARLY')
  assert.equal(earlyAclCycling.cycling.allowed, true)
  assert.equal(earlyAclCycling.recommendedSession, 'cycling')
  assert.equal(earlyAclCycling.flexion.modeId, 'isotonic')
  assert.ok(earlyAclCycling.safetyLocks.some((s) => s.includes('ACL Graft Precaution')))

  // Gate 2 (ROM >= 100) + Gate 4 Early + General -> 'both'
  const earlyGeneralBoth = computeClinicalPrescription({
    pathology: 'General',
    weeksPostOp: 4,
    painVas: 2,
    effusion: 'none',
    romExt: 0,
    romFlex: 115,
  })
  assert.equal(earlyGeneralBoth.protocolId, 'REC_CYC_EARLY')
  assert.equal(earlyGeneralBoth.recommendedSession, 'both')

  // Gate 4 Intermediate L1 (Weeks 6-9) -> REC_INTERM_L1
  const intermL1 = computeClinicalPrescription({
    pathology: 'Meniscus',
    weeksPostOp: 7,
    painVas: 2,
    effusion: 'trace',
    romExt: 0,
    romFlex: 120,
  })
  assert.equal(intermL1.protocolId, 'REC_INTERM_L1')
  assert.equal(intermL1.recommendedSession, 'both')
  assert.equal(intermL1.flexion.modeId, 'isokinetic')
  assert.equal(intermL1.flexion.targetTorqueNm, 45)
  assert.equal(intermL1.cycling.allowed, true)

  // Gate 4 Intermediate L2 (Weeks >= 10, ROM >= 125, VAS <= 1, none/trace) -> REC_INTERM_L2
  const intermL2 = computeClinicalPrescription({
    pathology: 'ACLR',
    weeksPostOp: 12,
    painVas: 1,
    effusion: 'none',
    romExt: 0,
    romFlex: 135,
    dOffsetCm: 9,
    lLegCm: 30,
  })
  assert.equal(intermL2.protocolId, 'REC_INTERM_L2')
  assert.equal(intermL2.recommendedSession, 'both')
  assert.equal(intermL2.flexion.modeId, 'isokinetic')
  assert.equal(intermL2.flexion.targetTorqueNm, 68)
  assert.equal(intermL2.geometry.gamma60, 1.26)
})

test('normalizeIntake validates clinical bounds', () => {
  assert.throws(() => normalizeIntake({ pathology: 'Invalid', weeksPostOp: 2, painVas: 2, effusion: 'none', romFlex: 100 }))
  assert.throws(() => normalizeIntake({ pathology: 'ACLR', weeksPostOp: 2, painVas: 2, effusion: 'none', romExt: 40, romFlex: 35 }))
  assert.throws(() => normalizeIntake({ pathology: 'ACLR', weeksPostOp: 2, painVas: 11, effusion: 'none', romFlex: 100 }))
  assert.throws(() => normalizeIntake({ pathology: 'ACLR', weeksPostOp: 2, painVas: 2, effusion: 'invalid', romFlex: 100 }))
})

test('AuthStore tracks first-signup needsIntake and persists clinical prescription across restart', async () => {
  const directory = await mkdtemp(path.join(tmpdir(), 'doula-clinical-'))
  try {
    const store = new AuthStore(directory)
    await store.init()
    const signup = store.startSignup('Aarav', 'aarav@gmail.com')
    const ticket = store.verifySignup(signup.email, signup.code)
    const { user } = await store.finishSignup(ticket, 'pass1234')

    assert.equal(user.needsIntake, true)
    assert.equal(user.intakeCompleted, false)
    assert.equal(user.prescription, null)

    const updated = await store.setClinicalIntake(user.id, {
      pathology: 'ACLR',
      weeksPostOp: 4,
      painVas: 2,
      effusion: 'trace',
      romExt: 0,
      romFlex: 110,
      bodyMassKg: 70,
    })

    assert.equal(updated.needsIntake, false)
    assert.equal(updated.intakeCompleted, true)
    assert.equal(updated.prescription.protocolId, 'REC_CYC_EARLY')

    const restarted = new AuthStore(directory)
    await restarted.init()
    const { user: reloaded } = await restarted.login('aarav@gmail.com', 'pass1234')
    assert.equal(reloaded.needsIntake, false)
    assert.equal(reloaded.clinicalIntake.romFlex, 110)
    assert.equal(reloaded.prescription.protocolId, 'REC_CYC_EARLY')
  } finally {
    await rm(directory, { recursive: true, force: true })
  }
})

test('HTTP server serves intake APIs and exposes clinical prescription in /api/device/config', async () => {
  const directory = await mkdtemp(path.join(tmpdir(), 'doula-http-clinical-'))
  const { server, store } = await createAppServer({ dataDir: directory, bypassEsp: true })
  await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve))
  const port = server.address().port
  const baseUrl = `http://127.0.0.1:${port}`

  try {
    // Sign up via store + HTTP finish -> clinical intake -> device config
    const { email, code } = store.startSignup('Riya', 'riya@gmail.com')

    const verifyRes = await fetch(`${baseUrl}/api/auth/verify`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, code }),
    })
    const verifyData = await verifyRes.json()
    assert.equal(verifyData.ok, true)
    const verifiedCookie = verifyRes.headers.getSetCookie().map((c) => c.split(';')[0]).join('; ')

    const finishRes = await fetch(`${baseUrl}/api/auth/finish`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', Cookie: verifiedCookie },
      body: JSON.stringify({ password: 'password123' }),
    })
    const sessionCookie = finishRes.headers
      .getSetCookie()
      .map((c) => c.split(';')[0])
      .find((c) => c.startsWith('doula_session='))
    assert.ok(sessionCookie)
    const finishData = await finishRes.json()
    assert.equal(finishData.user.needsIntake, true)

    // Submit clinical intake
    const intakeRes = await fetch(`${baseUrl}/api/clinical/intake`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', Cookie: sessionCookie },
      body: JSON.stringify({
        pathology: 'ACLR',
        weeksPostOp: 8,
        painVas: 2,
        effusion: 'trace',
        romExt: 0,
        romFlex: 120,
        bodyMassKg: 68,
      }),
    })
    const intakeData = await intakeRes.json()
    assert.equal(intakeRes.status, 200)
    assert.equal(intakeData.prescription.protocolId, 'REC_INTERM_L1')
    assert.equal(intakeData.user.needsIntake, false)

    // Verify /api/device/config returns prescription for selector, flexion, and cycling pages
    const cfgRes = await fetch(`${baseUrl}/api/device/config`, {
      headers: { Cookie: sessionCookie },
    })
    const cfgData = await cfgRes.json()
    assert.equal(cfgRes.status, 200)
    assert.equal(cfgData.bypassEsp, true)
    assert.equal(cfgData.prescription.protocolId, 'REC_INTERM_L1')
    assert.equal(cfgData.prescription.flexion.modeId, 'isokinetic')
    assert.equal(cfgData.clinicalIntake.romFlex, 120)

    // Verify protected clinical endpoints require auth
    const unauthRes = await fetch(`${baseUrl}/api/clinical/prescription`)
    assert.equal(unauthRes.status, 401)
  } finally {
    await new Promise((resolve) => server.close(resolve))
    await rm(directory, { recursive: true, force: true })
  }
})
