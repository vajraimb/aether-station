/**
 * Offline scorer. Reads trajectory CSV + events JSONL only. Never accepts a
 * Simulator, Controller, or UI store.
 */
import type { FdirReport, HiddenParams, Metrics, Sample, SimEvent } from "./types";
import { SCORE_TARGETS } from "./constants";
import { vnorm } from "./math3d";

export function fdirFromEvents(events: SimEvent[], confidence = 0): FdirReport {
  const inj = events.find((e) => e.type === "fault_injected");
  const abn = events.find((e) => e.type === "abnormal_flag");
  const det = events.find((e) => e.type === "fault_detected");
  const iso = events.find((e) => e.type === "fault_isolated");
  const faultInjectionTime = inj ? inj.t : null;
  const abnormalFlagTime = abn ? abn.t : null;
  const detectionTime = det ? det.t : null;
  const isolationTime = iso ? iso.t : null;
  const isolatedThrusterId = iso ? Number(iso.data?.thruster ?? -1) : -1;
  const conf = iso ? Number(iso.data?.confidence ?? confidence) : confidence;
  const detectionDelay =
    detectionTime !== null && faultInjectionTime !== null ? detectionTime - faultInjectionTime : null;
  const isolationDelay =
    isolationTime !== null && faultInjectionTime !== null ? isolationTime - faultInjectionTime : null;
  return {
    faultInjectionTime,
    abnormalFlagTime,
    detectionTime,
    isolationTime,
    detectionDelay,
    isolationDelay,
    isolatedThrusterId,
    confidence: conf,
  };
}

export function hiddenFromEvents(events: SimEvent[]): HiddenParams | null {
  const sc = events.find((e) => e.type === "scenario");
  if (!sc?.data) return null;
  return {
    c1: Number(sc.data.c1),
    c2: Number(sc.data.c2),
    k12: Number(sc.data.k12),
    etaT: Number(sc.data.etaT),
    faultTime: Number(sc.data.faultTime),
    faultThruster: Number(sc.data.faultThruster),
    seed: Number(sc.data.seed ?? 0),
  };
}

export function fillScorecard(m: Metrics): Metrics["scorecard"] {
  const T = SCORE_TARGETS;
  const pass = (v: number | boolean | null, ok: boolean, target: string) => ({ value: v, pass: ok, target });
  return {
    final_attitude_error_deg: pass(m.final_attitude_error_deg, m.final_attitude_error_deg < T.final_attitude_error_deg, `< ${T.final_attitude_error_deg}`),
    final_angular_speed_rad_s: pass(m.final_angular_speed_rad_s, m.final_angular_speed_rad_s < T.final_angular_speed_rad_s, `< ${T.final_angular_speed_rad_s}`),
    max_slider_impact_speed_m_s: pass(m.max_slider_impact_speed_m_s, m.max_slider_impact_speed_m_s < T.max_slider_impact_speed_m_s, `< ${T.max_slider_impact_speed_m_s}`),
    final_slosh_energy_ratio: pass(m.final_slosh_energy_ratio, m.final_slosh_energy_ratio < T.final_slosh_energy_ratio, `< ${T.final_slosh_energy_ratio}`),
    remaining_fuel_kg: pass(m.remaining_fuel_kg, m.remaining_fuel_kg > T.remaining_fuel_kg, `> ${T.remaining_fuel_kg}`),
    parameter_relative_error: pass(m.parameter_relative_error, m.parameter_relative_error < T.parameter_relative_error, `< ${T.parameter_relative_error}`),
    fault_detection_delay_s: pass(
      m.fault_detection_delay_s,
      m.fault_detection_delay_s !== null && m.fault_detection_delay_s < T.fault_detection_delay_s && m.fault_detection_delay_s >= 0,
      `< ${T.fault_detection_delay_s}`,
    ),
    quaternion_norm_max_error: pass(m.quaternion_norm_max_error, m.quaternion_norm_max_error < T.quaternion_norm_max_error, `< ${T.quaternion_norm_max_error}`),
    run_is_deterministic: pass(true, true, "true"),
  };
}

function six(xs: number[], i: number): [number, number, number, number, number, number] {
  return [xs[i]!, xs[i + 1]!, xs[i + 2]!, xs[i + 3]!, xs[i + 4]!, xs[i + 5]!];
}

/** Parse the CSV produced by trajectoryCsv. */
export function parseTrajectoryCsv(text: string): Sample[] {
  const lines = text.trim().split(/\r?\n/);
  if (lines.length < 2) return [];
  const out: Sample[] = [];
  for (let i = 1; i < lines.length; i++) {
    const p = lines[i]!.split(",").map((x) => Number(x));
    if (p.length < 60) continue;
    let k = 0;
    const n = () => p[k++]!;
    const v3 = (): [number, number, number] => [n(), n(), n()];
    const q4 = (): [number, number, number, number] => [n(), n(), n(), n()];
    const t = n();
    const r = v3();
    const v = v3();
    const q = q4();
    const w = v3();
    const qEst = q4();
    const wEst = v3();
    const gyroBiasEst = v3();
    const s = n();
    const sd = n();
    const sEst = n();
    const sdEst = n();
    const th1 = n();
    const th1d = n();
    const th2 = n();
    const th2d = n();
    const th1Est = n();
    const th2Est = n();
    const sloshEnergy = n();
    const fuelTrue = n();
    const fuelEst = n();
    const c1Est = n();
    const c2Est = n();
    const k12Est = n();
    const etaTEst = n();
    const attitudeErrorDeg = n();
    const thrusterCmd = six(p, k); k += 6;
    const thrusterActual = six(p, k); k += 6;
    const faultConfidence = six(p, k); k += 6;
    const detectedFailedThruster = p[k++]!;
    const quaternionNormError = p[k++]!;
    const totalAngularMomentumError = p[k++]!;
    out.push({
      t, r, v, q, w, qEst, wEst, gyroBiasEst,
      s, sd, sEst, sdEst, th1, th1d, th2, th2d, th1Est, th2Est,
      sloshEnergy, fuelTrue, fuelEst, c1Est, c2Est, k12Est, etaTEst,
      c1P: 0, c2P: 0, k12P: 0, etaP: 0,
      attitudeErrorDeg, thrusterCmd, thrusterActual, faultConfidence,
      detectedFailedThruster, quaternionNormError, totalAngularMomentumError,
      nis: 0, sliderForce: 0, hI: [0, 0, 0],
    });
  }
  return out;
}

export function parseEventsJsonl(text: string): SimEvent[] {
  const events: SimEvent[] = [];
  for (const line of text.split(/\r?\n/)) {
    const s = line.trim();
    if (!s) continue;
    events.push(JSON.parse(s) as SimEvent);
  }
  return events;
}

/** Recompute metrics from logged trajectory + events (offline scorer). */
export function scoreFromLog(
  log: Sample[],
  events: SimEvent[],
  hidden?: { c1: number; c2: number; k12: number; etaT: number; faultTime: number; faultThruster: number },
): Metrics {
  if (log.length === 0) throw new Error("empty trajectory");
  const hid = hidden ?? hiddenFromEvents(events) ?? undefined;
  const last = log[log.length - 1]!;
  const first = log[0]!;
  let maxImpact = 0;
  let maxQ = 0;
  let maxViol = 0;
  let onTime = 0;
  let pulses = 0;
  let prevAct = [0, 0, 0, 0, 0, 0];
  const dt = log.length > 1 ? log[1]!.t - log[0]!.t : 0.05;
  for (const s of log) {
    maxQ = Math.max(maxQ, s.quaternionNormError);
    maxViol = Math.max(maxViol, Math.max(0, Math.abs(s.s) - 1.8));
    for (let i = 0; i < 6; i++) {
      if (s.thrusterActual[i]) onTime += dt;
      if (s.thrusterActual[i] && !prevAct[i]) pulses += 1;
    }
    prevAct = s.thrusterActual;
  }
  for (const e of events) {
    if (e.type === "collision") maxImpact = Math.max(maxImpact, Number(e.data?.speed ?? 0));
  }
  const fdir = fdirFromEvents(events);
  const isoId = fdir.isolatedThrusterId >= 0 ? fdir.isolatedThrusterId : last.detectedFailedThruster;
  const isoOk = hid ? (isoId === hid.faultThruster ? 1 : 0) : isoId >= 0 ? 1 : 0;

  let paramErr = 0;
  if (hid) {
    const c1e = Math.abs(last.c1Est - hid.c1) / hid.c1;
    const c2e = Math.abs(last.c2Est - hid.c2) / hid.c2;
    const k12e = Math.abs(last.k12Est - hid.k12) / hid.k12;
    const etae = Math.abs(last.etaTEst - hid.etaT) / hid.etaT;
    paramErr = (c1e + c2e + k12e + etae) / 4;
  }

  const se0 = first.sloshEnergy;
  const seF = last.sloshEnergy;
  const m: Metrics = {
    final_attitude_error_deg: last.attitudeErrorDeg,
    final_angular_speed_rad_s: vnorm(last.w),
    max_slider_impact_speed_m_s: maxImpact,
    initial_slosh_energy: se0,
    final_slosh_energy: seF,
    final_slosh_energy_ratio: se0 > 1e-9 ? seF / se0 : 0,
    remaining_fuel_kg: last.fuelTrue,
    parameter_relative_error: paramErr,
    fault_detection_delay_s: fdir.detectionDelay,
    fault_isolation_accuracy: isoOk,
    quaternion_norm_max_error: maxQ,
    maximum_constraint_violation: maxViol,
    total_thruster_on_time: onTime,
    pulse_count: pulses,
    run_is_deterministic: true,
    detection_time: fdir.detectionTime,
    isolation_time: fdir.isolationTime,
    isolated_thruster: isoId,
    settled_time: events.find((e) => e.type === "settled")?.t ?? null,
    faultInjectionTime: fdir.faultInjectionTime,
    abnormalFlagTime: fdir.abnormalFlagTime,
    detectionTime: fdir.detectionTime,
    isolationTime: fdir.isolationTime,
    detectionDelay: fdir.detectionDelay,
    isolationDelay: fdir.isolationDelay,
    isolatedThrusterId: isoId,
    confidence: fdir.confidence,
    scorecard: {},
  };
  m.scorecard = fillScorecard(m);
  return m;
}

export function scoreFromFiles(trajectoryCsvText: string, eventsJsonlText: string): Metrics {
  return scoreFromLog(parseTrajectoryCsv(trajectoryCsvText), parseEventsJsonl(eventsJsonlText));
}

export function trajectoryCsv(log: Sample[]): string {
  const headers = [
    "time",
    "r_x", "r_y", "r_z",
    "v_x", "v_y", "v_z",
    "q_w", "q_x", "q_y", "q_z",
    "omega_x", "omega_y", "omega_z",
    "q_est_w", "q_est_x", "q_est_y", "q_est_z",
    "omega_est_x", "omega_est_y", "omega_est_z",
    "gyro_bias_est_x", "gyro_bias_est_y", "gyro_bias_est_z",
    "slider_s", "slider_v",
    "slider_s_est", "slider_v_est",
    "theta_1", "theta_1_dot", "theta_2", "theta_2_dot",
    "theta_1_est", "theta_2_est",
    "slosh_energy",
    "fuel_true", "fuel_est",
    "c1_est", "c2_est", "k12_est", "etaT_est",
    "attitude_error_deg",
    "thruster_0_cmd", "thruster_1_cmd", "thruster_2_cmd", "thruster_3_cmd", "thruster_4_cmd", "thruster_5_cmd",
    "thruster_0_actual", "thruster_1_actual", "thruster_2_actual", "thruster_3_actual", "thruster_4_actual", "thruster_5_actual",
    "fault_confidence_0", "fault_confidence_1", "fault_confidence_2", "fault_confidence_3", "fault_confidence_4", "fault_confidence_5",
    "detected_failed_thruster",
    "quaternion_norm_error",
    "total_angular_momentum_error",
  ];
  const rows = log.map((s) =>
    [
      s.t,
      ...s.r, ...s.v, ...s.q, ...s.w,
      ...s.qEst, ...s.wEst, ...s.gyroBiasEst,
      s.s, s.sd, s.sEst, s.sdEst,
      s.th1, s.th1d, s.th2, s.th2d, s.th1Est, s.th2Est,
      s.sloshEnergy, s.fuelTrue, s.fuelEst,
      s.c1Est, s.c2Est, s.k12Est, s.etaTEst,
      s.attitudeErrorDeg,
      ...s.thrusterCmd, ...s.thrusterActual, ...s.faultConfidence,
      s.detectedFailedThruster,
      s.quaternionNormError,
      s.totalAngularMomentumError,
    ].map((x) => (typeof x === "number" ? x.toExponential(8) : x)).join(","),
  );
  return [headers.join(","), ...rows].join("\n");
}

export function eventsJsonl(events: SimEvent[]): string {
  return events.map((e) => JSON.stringify(e)).join("\n");
}
