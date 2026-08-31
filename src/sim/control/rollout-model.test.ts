import { writeJson } from "../io";
import { defaultPublicConfig, Q0, THRUSTERS, W0 } from "../constants";
import {
  integrateWithCollision,
  massState,
} from "../dynamics";
import { qnormalize, vnorm, type Quat, type Vec3 } from "../math3d";
import { ThrusterSystem } from "../thrusters";
import type { Command, PublicConfig, SimState } from "../types";
import { generatePulsePrimitives, type PulsePrimitive } from "./discrete-actions";
import {
  PARITY_ROLLOUT_CONFIG,
  applyPrimitive,
  cloneRolloutState,
  geodesicAttitudeError,
  rolloutAdvance,
  rolloutFromSimLike,
  type RolloutParameters,
  type RolloutState,
} from "./rollout-model";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

function idleRollout(over: Partial<RolloutState> = {}): RolloutState {
  const cfg = defaultPublicConfig();
  const q = qnormalize(Q0);
  const ms = massState(cfg, 0.4, 0.1, -0.05, 5);
  return {
    time: 0,
    qBI: q,
    omegaB: [W0[0], W0[1], W0[2]],
    sliderS: 0.4,
    sliderV: 0.05,
    theta1: 0.1,
    theta1Dot: 0,
    theta2: -0.05,
    theta2Dot: 0.02,
    fuelMass: 5,
    pendingPulses: [],
    rCmI: [0, 0, 0],
    vCmI: [0, 0, 0],
    ...over,
  };
}

function belief(cfg: PublicConfig, eta = 0.873): RolloutParameters {
  const st = idleRollout();
  return {
    inertiaEstimate: massState(cfg, st.sliderS, st.theta1, st.theta2, st.fuelMass).Icm,
    etaTEstimate: eta,
    c1Estimate: 0.137,
    c2Estimate: 0.091,
    k12Estimate: 0.318,
    failedThrusterBeliefs: [],
  };
}

function toSim(s: RolloutState): SimState {
  return {
    t: s.time,
    rI: [0, 0, 0],
    vI: [0, 0, 0],
    rCmI: [...s.rCmI] as Vec3,
    vCmI: [...s.vCmI] as Vec3,
    q: [...s.qBI] as Quat,
    w: [...s.omegaB] as Vec3,
    s: s.sliderS,
    sd: s.sliderV,
    th1: s.theta1,
    th1d: s.theta1Dot,
    th2: s.theta2,
    th2d: s.theta2Dot,
    fuel: s.fuelMass,
  };
}

function plantOpenLoop(
  cfg: PublicConfig,
  start: RolloutState,
  sequence: { t: number; cmd: Command }[],
  duration: number,
  eta: number,
  c1: number,
  c2: number,
  k12: number,
): RolloutState {
  const thr = new ThrusterSystem(eta, {
    Fmax: cfg.maxThrust,
    delay: cfg.commandDelay,
    minPulse: cfg.minPulse,
    isp: cfg.isp,
    g0: cfg.g0,
  });
  let st = toSim(start);
  let nextCmd = 0;
  const dt = 0.005;
  while (st.t + 1e-12 < duration) {
    while (nextCmd < sequence.length && sequence[nextCmd]!.t <= st.t + 1e-12) {
      thr.submit(st.t, sequence[nextCmd]!.cmd);
      nextCmd += 1;
    }
    const snap = thr.evaluate(st.t, dt, st.fuel);
    const u = {
      FthrB: snap.Fb,
      tauThrO: snap.tauO,
      Fslider: sequence[0]?.cmd.sliderForce ?? 0,
      c1,
      c2,
      k12,
    };
    const col = integrateWithCollision(cfg, st, u, dt);
    st = col.state;
    st.fuel = Math.max(0, st.fuel + snap.fuelDot * dt);
  }
  return rolloutFromSimLike({
    time: st.t,
    q: st.q,
    w: st.w,
    s: st.s,
    sd: st.sd,
    th1: st.th1,
    th1d: st.th1d,
    th2: st.th2,
    th2d: st.th2d,
    fuel: st.fuel,
    rCmI: st.rCmI,
    vCmI: st.vCmI,
  });
}

function cmdOf(p: PulsePrimitive, slider = 0): Command {
  const pulseWidth: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
  for (const id of p.thrusterIds) pulseWidth[id] = p.durationS;
  return { sliderForce: slider, pulseWidth };
}

function err(a: RolloutState, b: RolloutState, cfg: PublicConfig) {
  const att = geodesicAttitudeError(a.qBI, b.qBI);
  const dw = vnorm([a.omegaB[0] - b.omegaB[0], a.omegaB[1] - b.omegaB[1], a.omegaB[2] - b.omegaB[2]]);
  const dfuel = Math.abs(a.fuelMass - b.fuelMass);
  const dth = Math.hypot(a.theta1 - b.theta1, a.theta2 - b.theta2);
  return { att, dw, dfuel, dth, t: Math.abs(a.time - b.time) };
}

export function runRolloutTests(): T[] {
  const out: T[] = [];
  const cfg = defaultPublicConfig({ duration: 10 });
  const params = belief(cfg);
  const all = generatePulsePrimitives(THRUSTERS);
  const coast = all.find((p) => p.thrusterIds.length === 0 && p.durationS === 0.32)!;
  const single = all.find((p) => p.thrusterIds.length === 1 && p.thrusterIds[0] === 0 && p.durationS === 0.16)!;
  const pair = all.find((p) => p.thrusterIds.length === 2 && p.thrusterIds[0] === 0 && p.thrusterIds[1] === 4 && p.durationS === 0.32)!;
  const rcfg = PARITY_ROLLOUT_CONFIG;

  {
    const s0 = idleRollout();
    const a = applyPrimitive(s0, params, cfg, coast, rcfg);
    const b = applyPrimitive(cloneRolloutState(s0), params, cfg, coast, rcfg);
    const e = err(a, b, cfg);
    check("test_rollout_determinism", e.att < 1e-15 && e.dw < 1e-15 && e.dfuel < 1e-15, `Δatt=${e.att} Δw=${e.dw}`, out);
  }

  {
    const s0 = idleRollout({ omegaB: [0.02, -0.01, 0.015] });
    const coasted = rolloutAdvance(s0, params, cfg, 1.0, rcfg);
    check("test_rollout_coast", coasted.time > 0.99 && Math.abs(coasted.fuelMass - s0.fuelMass) < 1e-9, `t=${coasted.time} fuel=${coasted.fuelMass}`, out);
  }

  {
    const s0 = idleRollout();
    const fired = applyPrimitive(s0, params, cfg, single, rcfg);
    check("test_rollout_single_thruster", fired.fuelMass < s0.fuelMass - 1e-6, `fuel ${s0.fuelMass}→${fired.fuelMass}`, out);
    check("test_rollout_delay", fired.pendingPulses.some((p) => Math.abs(p.tOn - cfg.commandDelay) < 1e-9), `pending=${JSON.stringify(fired.pendingPulses)}`, out);
  }

  {
    const s0 = idleRollout();
    const fired = applyPrimitive(s0, params, cfg, pair, rcfg);
    check("test_rollout_pair", fired.fuelMass < s0.fuelMass - 1e-5, `fuel Δ=${(s0.fuelMass - fired.fuelMass).toFixed(5)}`, out);
  }

  {
    const pending = [{ id: 1, tOn: 0.05, tOff: 0.25 }];
    const s0 = idleRollout({ pendingPulses: pending });
    const next = rolloutAdvance(s0, params, cfg, 0.5, rcfg);
    check("test_rollout_pending_pulse", next.fuelMass < s0.fuelMass, `fuel ${next.fuelMass}`, out);
  }

  {
    const failed: RolloutParameters = { ...params, failedThrusterBeliefs: [0] };
    const s0 = idleRollout();
    const a = applyPrimitive(s0, failed, cfg, single, rcfg);
    const b = applyPrimitive(cloneRolloutState(s0), failed, cfg, coast, rcfg);
    check("test_rollout_failed_thruster", Math.abs(a.fuelMass - b.fuelMass) < 1e-9, `fuel a=${a.fuelMass} b=${b.fuelMass}`, out);
  }

  {
    const s0 = idleRollout({ fuelMass: 2.85 });
    const fired = applyPrimitive(s0, params, cfg, pair, rcfg);
    check("test_rollout_fuel", fired.fuelMass < s0.fuelMass && fired.fuelMass > 2.7, `fuel=${fired.fuelMass}`, out);
  }

  {
    const s0 = idleRollout();
    const nq: Quat = [-s0.qBI[0], -s0.qBI[1], -s0.qBI[2], -s0.qBI[3]];
    const s1 = idleRollout({ qBI: nq });
    const a = applyPrimitive(s0, params, cfg, coast, rcfg);
    const b = applyPrimitive(s1, params, cfg, coast, rcfg);
    const attA = geodesicAttitudeError(a.qBI, cfg.qTarget);
    const attB = geodesicAttitudeError(b.qBI, cfg.qTarget);
    check("test_quaternion_sign_invariance", Math.abs(attA - attB) < 1e-9, `att ${attA} vs ${attB}`, out);
  }

  const cases: { name: string; prims: PulsePrimitive[]; duration: number }[] = [
    { name: "coast", prims: [coast], duration: 0.5 },
    { name: "single", prims: [single], duration: 1 },
    { name: "pair", prims: [pair], duration: 2 },
    { name: "delay-span", prims: [single, coast], duration: 5 },
  ];
  const rows: unknown[] = [];
  for (const c of cases) {
    const s0 = idleRollout();
    let roll = cloneRolloutState(s0);
    const seq: { t: number; cmd: Command }[] = [];
    let t = 0;
    for (const p of c.prims) {
      seq.push({ t, cmd: cmdOf(p) });
      roll = applyPrimitive(roll, params, cfg, p, rcfg);
      t += p.durationS;
    }
    if (c.duration > t) roll = rolloutAdvance(roll, params, cfg, c.duration - t, rcfg);
    const plant = plantOpenLoop(cfg, s0, seq, c.duration, params.etaTEstimate, params.c1Estimate, params.c2Estimate, params.k12Estimate);
    const e = err(roll, plant, cfg);
    rows.push({ case: c.name, duration: c.duration, attRad: e.att, omegaErr: e.dw, fuelErr: e.dfuel, sloshErr: e.dth, timeErr: e.t });
    const attOk = e.att < (c.duration <= 1 ? 0.02 : 0.08);
    const wOk = e.dw < (c.duration <= 1 ? 0.01 : 0.04);
    const fuelOk = e.dfuel < 0.02;
    check(`test_rollout_plant_parity:${c.name}`, attOk && wOk && fuelOk, `att=${e.att.toExponential(2)} dw=${e.dw.toExponential(2)} dfuel=${e.dfuel.toExponential(2)}`, out);
  }

  {
    const s0 = idleRollout({ pendingPulses: [{ id: 3, tOn: 0.0, tOff: 0.2 }] });
    let roll = rolloutAdvance(s0, params, cfg, 2, rcfg);
    const plant = plantOpenLoop(
      cfg,
      idleRollout(),
      [{ t: 0, cmd: { sliderForce: 0, pulseWidth: [0, 0, 0, 0.2, 0, 0] } }],
      2,
      params.etaTEstimate,
      params.c1Estimate,
      params.c2Estimate,
      params.k12Estimate,
    );
    const e = err(roll, plant, cfg);
    rows.push({ case: "pending-at-start", duration: 2, attRad: e.att, omegaErr: e.dw, fuelErr: e.dfuel, sloshErr: e.dth, timeErr: e.t });
    check("test_rollout_plant_parity:pending", e.att < 0.08 && e.dw < 0.04, `att=${e.att.toExponential(2)} dw=${e.dw.toExponential(2)}`, out);
  }

  writeJson("outputs/rollout-parity.json", {
    physicsBaselineSha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
    controllerVersion: "discrete-pulse-v2",
    notes:
      "Rollout uses the frozen rk4/collision kernel with belief parameters, 5 ms parity dt, and the plant delay queue. Thresholds separate implementation bugs from missing CM-translation cost and slider PD mismatch.",
    tolerances: { attRad_0p5s: 0.02, attRad_5s: 0.08, omega_0p5s: 0.01, omega_5s: 0.04, fuel: 0.02 },
    rows,
  });

  return out;
}
