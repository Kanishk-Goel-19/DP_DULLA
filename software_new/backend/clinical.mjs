// Automated Clinical Decision Tree (Section 3.2 & 3.3 of README_INTEGRATED_SYSTEM_AND_WEB_APP.md)

export const PATHOLOGIES = {
  ACLR: 'ACL Reconstruction (ACLR)',
  TKA: 'Total Knee Arthroplasty (TKA)',
  Meniscus: 'Meniscal Repair',
  PCL: 'PCL Reconstruction',
  PFPS: 'Patellofemoral Pain Syndrome (PFPS)',
  General: 'General Knee Strengthening',
}

export const EFFUSION_GRADES = ['none', 'trace', 'moderate', 'severe']

function num(value, fallback) {
  const n = Number(value)
  return Number.isFinite(n) ? n : fallback
}

export function normalizeIntake(raw) {
  if (!raw || typeof raw !== 'object') throw new Error('Invalid clinical intake submission.')
  const pathology = Object.hasOwn(PATHOLOGIES, raw.pathology) ? raw.pathology : ''
  if (!pathology) throw new Error('Select a valid pathology or rehabilitation condition.')

  const weeksPostOp = num(raw.weeksPostOp, NaN)
  if (!Number.isFinite(weeksPostOp) || weeksPostOp < 0 || weeksPostOp > 260) {
    throw new Error('Weeks post-op must be between 0 and 260.')
  }

  const painVas = Math.round(num(raw.painVas, NaN))
  if (!Number.isFinite(painVas) || painVas < 0 || painVas > 10) {
    throw new Error('Pain score (VAS) must be between 0 and 10.')
  }

  const effusion = String(raw.effusion || '').toLowerCase()
  if (!EFFUSION_GRADES.includes(effusion)) {
    throw new Error('Select a valid joint swelling grade.')
  }

  const romExt = Math.round(num(raw.romExt ?? 0, NaN))
  const romFlex = Math.round(num(raw.romFlex, NaN))
  if (!Number.isFinite(romExt) || romExt < 0 || romExt > 60) {
    throw new Error('Extension limit must be between 0° and 60°.')
  }
  if (!Number.isFinite(romFlex) || romFlex < 10 || romFlex > 150) {
    throw new Error('Pain-free max flexion must be between 10° and 150°.')
  }
  if (romFlex <= romExt) {
    throw new Error('Max flexion angle must be greater than extension limit.')
  }

  const bodyMassKg = Math.round(num(raw.bodyMassKg ?? 70, 70) * 10) / 10
  if (bodyMassKg < 25 || bodyMassKg > 250) {
    throw new Error('Body mass must be between 25 kg and 250 kg.')
  }

  const injuredLimb = raw.injuredLimb === 'Left' ? 'Left' : 'Right'
  const dOffsetCm = Math.round(num(raw.dOffsetCm ?? 9.0, 9.0) * 10) / 10
  const lLegCm = Math.round(num(raw.lLegCm ?? 30.0, 30.0) * 10) / 10
  if (dOffsetCm < 5 || dOffsetCm > 15) throw new Error('Pivot vertical offset d_offset must be 5–15 cm.')
  if (lLegCm < 15 || lLegCm > 50) throw new Error('Tibial pad distance L_leg must be 15–50 cm.')

  return {
    pathology,
    pathologyLabel: PATHOLOGIES[pathology],
    weeksPostOp: Math.round(weeksPostOp * 10) / 10,
    painVas,
    effusion,
    romExt,
    romFlex,
    bodyMassKg,
    injuredLimb,
    dOffsetCm,
    lLegCm,
    updatedAt: new Date().toISOString(),
  }
}

export function computeClinicalPrescription(rawIntake) {
  const intake = normalizeIntake(rawIntake)
  const { pathology, pathologyLabel, weeksPostOp, painVas, effusion, romExt, romFlex, bodyMassKg, dOffsetCm, lLegCm } = intake

  // Offset-pivot kinematic gain factor Γ(60°) = 1 + (d_offset / L_leg) * sin(60°)
  const gamma60 = Math.round((1 + (dOffsetCm / lLegCm) * Math.sin(Math.PI / 3)) * 1000) / 1000
  const isAclEarly = pathology === 'ACLR' && weeksPostOp <= 6
  const safetyLocks = []
  const decisionPath = []

  if (isAclEarly) {
    safetyLocks.push('ACL Graft Precaution (Weeks 0–6): Anterior shear taper active between 40° and 0°.')
  }
  if (pathology === 'PFPS') {
    safetyLocks.push('PFPS Curve Shaping: Peak patellofemoral load moderated across high flexion angles.')
  }

  // Gate 1: Pain VAS >= 6 OR Severe Swelling OR Weeks Post-Op < 1
  if (painVas >= 6 || effusion === 'severe' || weeksPostOp < 1) {
    decisionPath.push(`Gate 1 Triggered (Acute Protection): VAS ${painVas}/10, Effusion "${effusion}", ${weeksPostOp} wks post-op`)
    safetyLocks.unshift('Bottom-Tier Cycling Locked: Acute inflammation / protection phase requires controlled CPM.')

    const targetRomMax = Math.min(romFlex, painVas >= 7 ? 65 : 90)
    const flexion = {
      modeId: 'passive',
      modeName: 'Passive Assist Mode',
      clinicalLabel: 'Top Hub Mode 1 (Passive CPM)',
      role: 'support',
      roleLabel: 'Support Torque',
      opMode: 3,
      suggestedReps: 15,
      targetTorqueNm: 15,
      torqueRangeNm: '4–25 N·m (Spasm cutoff 15 N·m)',
      speedSpec: '1.0°–2.0°/s (0.17–0.33 RPM)',
      romWindowDeg: [romExt, Math.max(romExt + 10, targetRomMax)],
      pauseHoldSec: 5,
      safetyTaper: 'Spasm backoff enabled at 15 N·m (50 N pad force)',
      secondaryNote: 'Gentle continuous passive motion to reduce edema and prevent arthrofibrosis.',
    }
    const cycling = {
      allowed: false,
      priorityLabel: 'Contraindicated / Locked (Acute Phase)',
      modeName: 'Passive Recovery Only',
      targetTorqueNm: 0,
      torqueRangeNm: 'Locked (< 1 wk or high pain/effusion)',
      cadenceRangeRpm: '—',
      stepperAngleDeg: '0°',
      powerRangeW: '0 W',
      durationMin: 0,
      suggestedCycles: 0,
      note: 'Defer full-revolution cycling until pain VAS < 6, swelling subsides, and flexion reaches ≥ 100°.',
    }
    return buildResult({
      intake,
      gamma60,
      protocolId: 'REC_CPM',
      phase: 'Phase 1A: Acute Post-Op Protection',
      headline: 'Passive CPM Protection (Flexion & Extension)',
      summary: `Based on ${pathologyLabel} (${weeksPostOp} wks, VAS ${painVas}/10, ${effusion} effusion), Gate 1 prescribes slow motor-assisted Passive CPM to protect healing tissue and reduce edema.`,
      recommendedSession: 'flexion',
      decisionPath,
      safetyLocks,
      flexion,
      cycling,
      bodyMassKg,
    })
  }

  decisionPath.push(`Gate 1 Cleared: VAS ${painVas}/10 (< 6), Effusion "${effusion}", ${weeksPostOp} wks post-op (≥ 1)`)

  // Gate 2: Current Max Flexion >= 100°
  if (romFlex < 100) {
    decisionPath.push(`Gate 2: Max Flexion ${romFlex}° (< 100° required for 360° pedal revolution)`)
    safetyLocks.unshift(`Cycling Contraindicated: Current flexion ${romFlex}° is below the 100° minimum cycle criterion.`)

    // Gate 3: Weeks Post-Op < 3 OR VAS >= 4
    if (weeksPostOp < 3 || painVas >= 4) {
      decisionPath.push(`Gate 3 Triggered: ${weeksPostOp} wks (< 3) or VAS ${painVas} (≥ 4) -> Dual CPM + Isometric Protocol`)
      const flexion = {
        modeId: 'passive',
        modeName: 'Passive Assist Mode',
        clinicalLabel: 'Top Hub Mode 1 (CPM +60°/90° Isometric)',
        role: 'support',
        roleLabel: 'Support Torque',
        opMode: 3,
        suggestedReps: 10,
        targetTorqueNm: 18,
        torqueRangeNm: '15–25 N·m (Gearbox 19–32 N·m)',
        speedSpec: '1.5°–2.4°/s CPM + 5s Isometric Hold @ 60° & 90°',
        romWindowDeg: [romExt, Math.min(105, romFlex + 5)],
        pauseHoldSec: 5,
        safetyTaper: 'Expand safe flexion +5° toward 105° target; avoid open-chain shear.',
        secondaryNote: 'Combines CPM ROM expansion (+5°) with pain-free multi-angle quad activation.',
      }
      const cycling = {
        allowed: false,
        priorityLabel: `Locked until Flexion ≥ 100° (currently ${romFlex}°)`,
        modeName: 'Pendulum Rocking / Deferred',
        targetTorqueNm: 5,
        torqueRangeNm: '≤ 5 N·m (unlocked once ROM ≥ 100°)',
        cadenceRangeRpm: '0–25 (rocking)',
        stepperAngleDeg: '0°–5°',
        powerRangeW: '< 15 W',
        durationMin: 5,
        suggestedCycles: 8,
        note: `Build flexion from ${romFlex}° toward 100°+ on Flexion & Extension before full cycling revolutions.`,
      }
      return buildResult({
        intake,
        gamma60,
        protocolId: 'REC_ISO_CPM',
        phase: 'Phase 1B: Early Protected Activation',
        headline: 'Top-Hub Passive CPM & Isometric Activation',
        summary: `With ${romFlex}° flexion (< 100°) and ${weeksPostOp < 3 ? 'early timeline (< 3 wks)' : `moderate pain (VAS ${painVas})`}, focus on Flexion & Extension Passive CPM (+5° ROM goal) alongside sub-maximal isometrics.`,
        recommendedSession: 'flexion',
        decisionPath,
        safetyLocks,
        flexion,
        cycling,
        bodyMassKg,
      })
    }

    // Gate 3 = NO: Early Physiological Curve-Shaped Isotonic
    decisionPath.push(`Gate 3 Cleared (${weeksPostOp} wks ≥ 3, VAS ${painVas} ≤ 3) -> Early Physiological Isotonic`)
    const flexion = {
      modeId: 'isotonic',
      modeName: 'Physiological Curve-Shaped Isotonic',
      clinicalLabel: 'Top Hub Mode 2 (Early Physiological Isotonic)',
      role: 'resistance',
      roleLabel: 'Resistance Torque',
      opMode: 4,
      suggestedReps: 12,
      targetTorqueNm: 20,
      torqueRangeNm: 'Ext peak 20 N·m (Gbx 25.2 N·m) · Flex 12 N·m',
      speedSpec: 'Controlled 45°/s curve-shaped resistance',
      romWindowDeg: [romExt, romFlex],
      pauseHoldSec: 1,
      safetyTaper: 'ACL-Safe Taper active (40°→0° capped)',
      secondaryNote: 'Angle-mapped isotonic loading while progressively regaining 100°+ knee flexion.',
    }
    const cycling = {
      allowed: false,
      priorityLabel: `Locked until Flexion ≥ 100° (currently ${romFlex}°)`,
      modeName: 'Zero-Load Coast (Deferred)',
      targetTorqueNm: 5,
      torqueRangeNm: '≤ 5 N·m once 100° achieved',
      cadenceRangeRpm: '30–40',
      stepperAngleDeg: '0°–5°',
      powerRangeW: '10–25 W',
      durationMin: 8,
      suggestedCycles: 10,
      note: `Flexion is currently ${romFlex}°. Reach ≥ 100° on Flexion & Extension before unlocking full 360° cycling.`,
    }
    return buildResult({
      intake,
      gamma60,
      protocolId: 'REC_EARLY_ISO',
      phase: 'Phase 1B–2A Transition: Early Curve Isotonic',
      headline: 'Physiological Curve-Shaped Isotonic (Flexion & Extension)',
      summary: `Pain is well controlled (VAS ${painVas}/10), though flexion (${romFlex}°) is still under 100°. Gate 3 prescribes Early Physiological Curve Isotonic (20 N·m peak extension with 40°→0° ACL taper) to build strength and ROM toward cycling clearance.`,
      recommendedSession: 'flexion',
      decisionPath,
      safetyLocks,
      flexion,
      cycling,
      bodyMassKg,
    })
  }

  // Gate 2 = YES: ROM >= 100° (Cleared for Cycling!)
  decisionPath.push(`Gate 2 Cleared: Max Flexion ${romFlex}° (≥ 100° — Cleared for Full-Revolution Cycling)`)
  safetyLocks.unshift(`Cycling Unlocked: ${romFlex}° pain-free flexion meets the ≥ 100° ergonomic crank criterion.`)

  // Gate 4: Rehab Phase & Weeks Post-Op
  if (weeksPostOp < 6) {
    decisionPath.push(`Gate 4 (Weeks ${weeksPostOp} < 6): Early Active Mobility & Endurance -> Combined Early Protocol`)
    const preferredSession = isAclEarly ? 'cycling' : 'both'
    const flexTorque = painVas >= 3 ? 20 : 22
    const flexion = {
      modeId: 'isotonic',
      modeName: 'Physiological Curve-Shaped Isotonic',
      clinicalLabel: 'Top Hub Mode 2 (Protected Curve Isotonic)',
      role: 'resistance',
      roleLabel: 'Resistance Torque',
      opMode: 4,
      suggestedReps: 12,
      targetTorqueNm: flexTorque,
      torqueRangeNm: `Peak Ext 20–25 N·m (Gbx ≤ 31.5 N·m) · Flex 12–15 N·m`,
      speedSpec: 'Controlled physiological curve · ACL taper 40°→0°',
      romWindowDeg: [romExt, romFlex],
      pauseHoldSec: 1,
      safetyTaper: 'ACL-Safe anterior shear taper (40°→0°)',
      secondaryNote: 'Use alongside Bottom-Tier Cycling for early neuromuscular coordination.',
    }
    const cycling = {
      allowed: true,
      priorityLabel: 'Cleared · Primary CKC Mobility Option',
      modeName: 'Light Constant Torque Cycling',
      targetTorqueNm: painVas >= 3 ? 8 : 14,
      torqueRangeNm: '6–22 N·m (Light Constant Torque)',
      cadenceRangeRpm: '35–55 RPM',
      stepperAngleDeg: '5°–18°',
      powerRangeW: '25–85 W',
      durationMin: 12,
      suggestedCycles: 18,
      note: 'Low-shear closed-kinetic-chain pedaling promotes synovial diffusion and symmetrical ROM.',
    }
    return buildResult({
      intake,
      gamma60,
      protocolId: 'REC_CYC_EARLY',
      phase: 'Phase 2A: Early-to-Intermediate Transition',
      headline: isAclEarly
        ? 'Prioritize Light Constant-Torque Cycling + Protected Curve Isotonic'
        : 'Combined Protocol: Light Cycling + Protected Curve Isotonic',
      summary: `Flexion (${romFlex}°) clears cycling. In Weeks 1–5 (${pathologyLabel}), Gate 4 recommends combining Light Constant-Torque Cycling (6–22 N·m, 35–55 RPM) with Protected Curve-Shaped Isotonic (${flexTorque} N·m peak).`,
      recommendedSession: preferredSession,
      decisionPath,
      safetyLocks,
      flexion,
      cycling,
      bodyMassKg,
    })
  }

  // Weeks 6–14+: Gate 4 Intermediate Strength & Power
  const isLevel2 = weeksPostOp >= 10 && romFlex >= 125 && painVas <= 1 && (effusion === 'none' || effusion === 'trace')
  const protocolId = isLevel2 ? 'REC_INTERM_L2' : 'REC_INTERM_L1'
  const phase = isLevel2
    ? 'Phase 2C: Intermediate Strengthening L2 (High-Load)'
    : 'Phase 2B: Intermediate Strengthening L1 (Continuous Rated)'

  decisionPath.push(
    isLevel2
      ? `Gate 4 (Weeks ${weeksPostOp} ≥ 10, ROM ${romFlex}° ≥ 125°, VAS ${painVas} ≤ 1): Combined Intermediate L2 Protocol`
      : `Gate 4 (Weeks ${weeksPostOp} ≥ 6, ROM ${romFlex}° ≥ 100°): Combined Intermediate L1 Protocol`
  )

  const targetKneeCap = isLevel2 ? 68 : 45
  const targetGbxTorque = Math.round(targetKneeCap * gamma60 * 10) / 10

  const flexion = {
    modeId: 'isokinetic',
    modeName: 'Isokinetic Mode',
    clinicalLabel: `Top Hub Mode 3 (Accommodating Isokinetic @ ${isLevel2 ? '60°/90°/120°/s' : '60°/s'})`,
    role: 'resistance',
    roleLabel: 'Resistance Torque',
    opMode: 5,
    suggestedReps: 10,
    targetTorqueNm: targetKneeCap,
    torqueRangeNm: isLevel2 ? '45–68 N·m Cap (Gbx ≤ 85.7 N·m)' : '35–45 N·m Cap (Gbx ≤ 56.7 N·m)',
    speedSpec: isLevel2 ? 'ω_iso = 60°/s, 90°/s & 120°/s' : 'ω_iso = 60°/s & 90°/s',
    romWindowDeg: [romExt, romFlex],
    pauseHoldSec: 1,
    safetyTaper: `Dynamic speed cap · Gearbox equivalent ${targetGbxTorque} N·m`,
    secondaryNote: 'Pair with Mode 2 Curve Isotonic for hypertrophy and H/Q ratio optimization.',
  }
  const cycling = {
    allowed: true,
    priorityLabel: 'Cleared · Ergometer Conditioning',
    modeName: isLevel2 ? 'High-Power / Isokinetic Cycling' : 'Constant Power Ergometer',
    targetTorqueNm: isLevel2 ? 42 : 28,
    torqueRangeNm: isLevel2 ? '≤ 55 N·m cont (85 N·m pk)' : '20–40 N·m continuous',
    cadenceRangeRpm: isLevel2 ? '60–75 RPM' : '50–65 RPM',
    stepperAngleDeg: isLevel2 ? '25°–42°' : '15°–28°',
    powerRangeW: isLevel2 ? '120–250 W' : '50–110 W',
    durationMin: isLevel2 ? 20 : 15,
    suggestedCycles: 25,
    note: 'Progressive aerobic and power conditioning while monitoring bilateral pedal symmetry (target LSI ≥ 80%).',
  }

  return buildResult({
    intake,
    gamma60,
    protocolId,
    phase,
    headline: 'Combined Intermediate Strength: Constant-Power Cycling + Isokinetic Knee Control',
    summary: `With ${romFlex}° flexion and ${weeksPostOp} weeks post-op, Gate 4 clears the Combined Intermediate Protocol: Constant-Power Cycling (${cycling.powerRangeW}) and Accommodating Isokinetic (${targetKneeCap} N·m cap, ${flexion.speedSpec}).`,
    recommendedSession: 'both',
    decisionPath,
    safetyLocks,
    flexion,
    cycling,
    bodyMassKg,
  })
}

function buildResult({
  intake,
  gamma60,
  protocolId,
  phase,
  headline,
  summary,
  recommendedSession,
  decisionPath,
  safetyLocks,
  flexion,
  cycling,
  bodyMassKg,
}) {
  return {
    protocolId,
    phase,
    headline,
    summary,
    recommendedSession,
    decisionPath,
    safetyLocks,
    flexion,
    cycling,
    geometry: {
      bodyMassKg,
      dOffsetCm: intake.dOffsetCm,
      lLegCm: intake.lLegCm,
      gamma60,
      normalizedTargetNmKg: Math.round((flexion.targetTorqueNm / bodyMassKg) * 100) / 100,
    },
    generatedAt: new Date().toISOString(),
  }
}
