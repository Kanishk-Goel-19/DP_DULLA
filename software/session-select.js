(() => {
  const banner = document.getElementById('clinicalBanner');
  const cyclingCard = document.getElementById('choose-cycling');
  const flexionCard = document.getElementById('choose-flexion');
  const cyclingPill = document.getElementById('cyclingRecPill');
  const flexionPill = document.getElementById('flexionRecPill');
  const cyclingBox = document.getElementById('cyclingRecBox');
  const flexionBox = document.getElementById('flexionRecBox');

  function el(tag, className, text) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (text != null) node.textContent = text;
    return node;
  }

  function clearAndShow(node) {
    if (!node) return;
    node.textContent = '';
    node.hidden = false;
  }

  function renderMissingIntake() {
    clearAndShow(banner);
    const copy = el('div', 'clinical-banner-copy');
    copy.append(
      el('span', 'clinical-banner-eyebrow', 'Clinical Decision Tree · U1: ESP_MASTER'),
      el('h2', 'clinical-banner-title', 'Complete your Clinical Intake Questionnaire'),
      el(
        'p',
        'clinical-banner-summary',
        'Answer 7 short questions (pathology, weeks post-op, VAS pain, knee ROM, effusion, weight) so DOULA can suggest exact sessions, mode, reps, and torque before you start.'
      )
    );
    const link = el('a', 'clinical-banner-link', 'Open Clinical Intake →');
    link.href = '/intake';
    banner.append(copy, link);
  }

  function buildRxBox(box, label, modeTitle, chips, note, caution) {
    clearAndShow(box);
    box.classList.toggle('is-caution', Boolean(caution));

    const head = el('div', 'rx-box-head');
    head.append(el('span', 'rx-box-label', label), el('span', 'rx-box-mode', modeTitle));
    box.appendChild(head);

    if (chips.length) {
      const row = el('div', 'rx-box-metrics');
      chips.forEach((c) => row.appendChild(el('span', 'rx-chip', c)));
      box.appendChild(row);
    }

    if (note) {
      box.appendChild(el('p', 'rx-box-note', note));
    }
  }

  function renderPrescription(rx, intake) {
    clearAndShow(banner);
    const copy = el('div', 'clinical-banner-copy');
    const recLabel =
      rx.recommendedSession === 'cycling'
        ? 'Primary: Cycling First + Protected Flexion'
        : rx.recommendedSession === 'both'
          ? 'Primary: Flexion & Cycling Paired'
          : 'Primary: Flexion & Extension First';

    copy.append(
      el('span', 'clinical-banner-eyebrow', `Clinical Decision Tree · ${rx.phase}`),
      el('h2', 'clinical-banner-title', `${rx.protocolId}: ${rx.headline}`),
      el('p', 'clinical-banner-summary', `${rx.summary}`)
    );

    const pillsWrap = el('div', 'clinical-banner-pills');
    pillsWrap.appendChild(el('span', 'clinical-banner-tag', recLabel));
    if (intake && intake.romFlex != null) {
      pillsWrap.appendChild(el('span', 'clinical-banner-tag', `Knee ROM ${intake.romFlex}° · VAS ${intake.painVas}/10`));
    }
    if (rx.safetyLocks && rx.safetyLocks.length) {
      rx.safetyLocks.forEach((lock) => {
        const isCaution = /Locked|Contraindicated|Precaution/i.test(lock);
        pillsWrap.appendChild(el('span', isCaution ? 'clinical-banner-tag is-lock' : 'clinical-banner-tag', lock));
      });
    }
    const editLink = el('a', 'clinical-banner-link', 'Update Intake →');
    editLink.href = '/intake';
    pillsWrap.appendChild(editLink);

    banner.append(copy, pillsWrap);

    if (flexionPill && rx.flexion) {
      flexionPill.hidden = false;
      flexionPill.textContent =
        rx.recommendedSession === 'flexion'
          ? '★ Recommended First'
          : rx.recommendedSession === 'both'
            ? '★ Recommended Core'
            : 'Paired Option';
      flexionCard?.classList.toggle('is-recommended', rx.recommendedSession === 'flexion' || rx.recommendedSession === 'both');
      const flx = rx.flexion;
      buildRxBox(
        flexionBox,
        'Suggested Flexion Settings',
        `${flx.modeName} (${flx.roleLabel})`,
        [
          `${flx.targetTorqueNm} Nm (${flx.torqueRangeNm})`,
          `${flx.suggestedReps} reps`,
          `ROM ${flx.romWindowDeg[0]}°–${flx.romWindowDeg[1]}°`,
        ],
        flx.safetyTaper || `${flx.speedSpec} · ${flx.secondaryNote}`,
        false
      );
    }

    if (cyclingPill && rx.cycling) {
      const cyc = rx.cycling;
      cyclingPill.hidden = false;
      cyclingPill.classList.toggle('is-caution', !cyc.allowed);
      cyclingCard?.classList.toggle('is-caution', !cyc.allowed);
      cyclingCard?.classList.toggle(
        'is-recommended',
        Boolean(cyc.allowed && (rx.recommendedSession === 'cycling' || rx.recommendedSession === 'both'))
      );

      cyclingPill.textContent = cyc.allowed
        ? rx.recommendedSession === 'cycling'
          ? '★ Recommended First'
          : rx.recommendedSession === 'both'
            ? '★ Recommended Pairing'
            : 'Cleared for Session'
        : 'Clinically Held (< 100° ROM / Acute)';

      const cycChips = cyc.allowed
        ? [
            `${cyc.targetTorqueNm} Nm (${cyc.torqueRangeNm})`,
            `${cyc.cadenceRangeRpm} RPM`,
            `${cyc.powerRangeW} · ${cyc.durationMin} min`,
          ]
        : ['Hold Cycling', 'Start Flexion & Extension →'];

      buildRxBox(
        cyclingBox,
        'Suggested Cycling Settings',
        cyc.allowed ? cyc.modeName : cyc.priorityLabel,
        cycChips,
        cyc.note,
        !cyc.allowed
      );
    }
  }

  async function init() {
    try {
      const res = await fetch('/api/device/config', {
        credentials: 'same-origin',
        cache: 'no-store',
      });
      if (!res.ok) return;
      const data = await res.json();
      if (data.prescription) {
        renderPrescription(data.prescription, data.clinicalIntake || null);
      } else {
        renderMissingIntake();
      }
    } catch {
      // Ignore transient load error cleanly; core session links remain functional
    }
  }

  init();
})();
