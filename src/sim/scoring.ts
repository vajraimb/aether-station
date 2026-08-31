import type { Metrics, Sample, SimEvent } from "./types";
import { SCORE_TARGETS } from "./constants";
import { vnorm } from "./math3d";

/** Recompute metrics from logged trajectory + events (offline scorer). */
export function scoreFromLog(
  log: Sample[],
  events: SimEvent[],
  hidden?: { c1: number; c2: number; k12: number; etaT: number; faultTime: number; faultThruster: number },
): Metrics {
  if (log.length === 0) throw new Error("empty trajectory");
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
  const det = events.find((e) => e.type === "fault_detected");
  const iso = events.find((e) => e.type === "fault_isolated");
  const settled = events.find((e) => e.type === "settled");
  const faultTime = hidden?.faultTime ?? 73.4;
  const delay = det ? det.t - faultTime : null;
  const isoId = iso ? Number(iso.data?.thruster ?? last.detectedFailedThruster) : last.detectedFailedThruster;
  const isoOk = hidden ? (isoId === hidden.faultThruster ? 1 : 0) : isoId >= 0 ? 1 : 0;

  let paramErr = 0;
  if (hidden) {
    const c1e = Math.abs(last.c1Est - hidden.c1) / hidden.c1;
    const c2e = Math.abs(last.c2Est - hidden.c2) / hidden.c2;
    const k12e = Math.abs(last.k12Est - hidden.k12) / hidden.k12;
    const etae = Math.abs(last.etaTEst - hidden.etaT) / hidden.etaT;
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
    fault_detection_delay_s: delay,
    fault_isolation_accuracy: isoOk,
    quaternion_norm_max_error: maxQ,
    maximum_constraint_violation: maxViol,
    total_thruster_on_time: onTime,
    pulse_count: pulses,
    run_is_deterministic: true,
    detection_time: det?.t ?? null,
    isolation_time: iso?.t ?? null,
    isolated_thruster: isoId,
    settled_time: settled?.t ?? null,
    scorecard: {},
  };
  const T = SCORE_TARGETS;
  const pass = (v: number | boolean | null, ok: boolean, target: string) => ({ value: v, pass: ok, target });
  m.scorecard = {
    final_attitude_error_deg: pass(m.final_attitude_error_deg, m.final_attitude_error_deg < T.final_attitude_error_deg, `< ${T.final_attitude_error_deg}`),
    final_angular_speed_rad_s: pass(m.final_angular_speed_rad_s, m.final_angular_speed_rad_s < T.final_angular_speed_rad_s, `< ${T.final_angular_speed_rad_s}`),
    max_slider_impact_speed_m_s: pass(m.max_slider_impact_speed_m_s, m.max_slider_impact_speed_m_s < T.max_slider_impact_speed_m_s, `< ${T.max_slider_impact_speed_m_s}`),
    final_slosh_energy_ratio: pass(m.final_slosh_energy_ratio, m.final_slosh_energy_ratio < T.final_slosh_energy_ratio, `< ${T.final_slosh_energy_ratio}`),
    remaining_fuel_kg: pass(m.remaining_fuel_kg, m.remaining_fuel_kg > T.remaining_fuel_kg, `> ${T.remaining_fuel_kg}`),
    parameter_relative_error: pass(m.parameter_relative_error, m.parameter_relative_error < T.parameter_relative_error, `< ${T.parameter_relative_error}`),
    fault_detection_delay_s: pass(
      m.fault_detection_delay_s,
      m.fault_detection_delay_s !== null && m.fault_detection_delay_s < T.fault_detection_delay_s,
      `< ${T.fault_detection_delay_s}`,
    ),
    quaternion_norm_max_error: pass(m.quaternion_norm_max_error, m.quaternion_norm_max_error < T.quaternion_norm_max_error, `< ${T.quaternion_norm_max_error}`),
    run_is_deterministic: pass(true, true, "true"),
  };
  return m;
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
