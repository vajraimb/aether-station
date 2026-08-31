import {
  attitudeErrorAngle,
  attitudeErrorVector,
  clamp,
  deg,
  qnormalize,
  vadd,
  vdot,
  vnorm,
  vscale,
  vsub,
  type Vec3,
} from "./math3d";
import { Estimator, type Estimate } from "./estimator";
import { allocateTorque, pulseImpulse, sliderForceCommand, torqueColumns } from "./allocate";
import { FdirEngine, type FdirSnapshot } from "./fdir";
import { CTRL_DT, MIN_PULSE } from "./constants";
import type { Command, Observation, PublicConfig } from "./types";

/**
 * Observation-only GNC. Sees ONLY a serialised Observation. Never holds a
 * Simulator, never reads PrivateScenario, never keys off a wall-clock fault
 * time or a hard-coded thruster identity.
 */
export class AgentController {
  readonly name: string = "observation";
  readonly estimator: Estimator;
  private cfg: PublicConfig;
  readonly fdir = new FdirEngine();
  private lastCmd: Command = { sliderForce: 0, pulseWidth: [0, 0, 0, 0, 0, 0] };
  private lastW: Vec3 = [0, 0, 0];
  private lastObs: Observation | null = null;
  private Hacc: Vec3 = [0, 0, 0];
  private lastFireT = -1e9;

  constructor(cfg: PublicConfig) {
    this.cfg = cfg;
    this.estimator = new Estimator(cfg);
  }

  get estimate(): Estimate {
    return this.estimator.snapshot();
  }
  getEstimate(): Estimate {
    return this.estimator.snapshot();
  }
  getFdir(): FdirSnapshot {
    return this.fdir.snapshot();
  }
  get faultConfidence() {
    return this.fdir.faultConfidence;
  }
  get detectedFailedThruster() {
    return this.fdir.detectedFailedThruster;
  }
  get detectionTime() {
    return this.fdir.detectionTime;
  }
  get isolationTime() {
    return this.fdir.isolationTime;
  }
  get isolationConfidence() {
    return this.fdir.isolationConfidence;
  }
  get isolated() {
    return this.fdir.isolated;
  }

  step(obs: Observation): Command {
    this.estimator.update(obs, null);
    const est = this.getEstimate();
    const dt = Math.max(
      1e-3,
      obs.timestamp - (this.lastObs?.timestamp ?? obs.timestamp - CTRL_DT),
    );
    const alpha: Vec3 = [
      (est.w[0] - this.lastW[0]) / dt,
      (est.w[1] - this.lastW[1]) / dt,
      (est.w[2] - this.lastW[2]) / dt,
    ];
    this.lastW = [est.w[0], est.w[1], est.w[2]];
    const probe = this.fdir.update(obs, this.cfg);
    this.lastObs = obs;
    const cmd = this.control(obs.timestamp, est, probe);
    this.lastCmd = cmd;
    this.fdir.pushCommand(obs.timestamp, cmd);
    this.estimator.updateEta(vnorm(this.predictedTorque(cmd.pulseWidth, est)), vnorm(alpha), 620);
    return cmd;
  }

  private predictedTorque(pulse: number[], est: Estimate): Vec3 {
    const cols = torqueColumns(this.cfg, est.s, est.th1, est.th2, est.fuel, this.estimator.etaT);
    let tau: Vec3 = [0, 0, 0];
    for (let i = 0; i < 6; i++) {
      const w = pulse[i] ?? 0;
      if (w <= 0) continue;
      tau = vadd(tau, vscale(cols[i]!, w / CTRL_DT));
    }
    return tau;
  }

  private control(
    t: number,
    est: Estimate,
    probe: { probeId: number; probe: boolean },
  ): Command {
    const Fs = sliderForceCommand(est.s, est.sd, this.cfg);
    const pulse: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
    const q = qnormalize(est.q);
    const w = est.w;
    const attErr = attitudeErrorVector(q, this.cfg.qTarget);
    const attDeg = deg(attitudeErrorAngle(q, this.cfg.qTarget));
    const wmag = vnorm(w);
    const cols = torqueColumns(this.cfg, est.s, est.th1, est.th2, est.fuel, this.estimator.etaT);
    const isolated = this.fdir.isolated;

    if (probe.probe && this.fdir.detectedFailedThruster < 0 && !isolated.has(probe.probeId)) {
      pulse[probe.probeId] = CTRL_DT;
    }

    const rateOnly = wmag > 0.08;
    let tauDes: Vec3;
    if (rateOnly) {
      tauDes = [
        -210 * w[0] - 20 * attErr[0],
        -210 * w[1] - 20 * attErr[1],
        -210 * w[2] - 20 * attErr[2],
      ];
    } else {
      let KpA: number;
      let KdA: number;
      if (attDeg < 4) {
        KpA = attDeg < 1.5 ? 14 : 24;
        KdA = 175;
      } else if (wmag < 0.02) {
        KpA = attDeg > 6 ? 48 : 36;
        KdA = 60;
      } else {
        KpA = attDeg > 25 ? 28 : 18;
        KdA = 95;
      }
      if (isolated.size > 0 && attDeg < 8) {
        KpA *= 0.85;
        KdA *= 1.1;
      }
      tauDes = [
        -KpA * 2 * attErr[0] - KdA * w[0],
        -KpA * 2 * attErr[1] - KdA * w[1],
        -KpA * 2 * attErr[2] - KdA * w[2],
      ];
    }

    if (this.cfg.fluidPresent) {
      const se = Math.abs(est.th1d) + Math.abs(est.th2d);
      if (se > 0.15) tauDes = vadd(tauDes, vscale(w, -15));
    }

    const tGo = Math.max(3, this.cfg.duration - t);
    const coastDeg = deg(vnorm([
      2 * attErr[0] + w[0] * Math.min(tGo, 8),
      2 * attErr[1] + w[1] * Math.min(tGo, 8),
      2 * attErr[2] + w[2] * Math.min(tGo, 8),
    ]));

    if (attDeg < 0.38 && wmag < 0.0018 && coastDeg < 0.8) {
      this.Hacc = vscale(this.Hacc, 0.8);
      return { sliderForce: Fs, pulseWidth: pulse };
    }

    const terminal = attDeg < 5 && wmag < 0.035 && !rateOnly;
    if (terminal) {
      this.Hacc = vadd(this.Hacc, vscale(tauDes, CTRL_DT));
      this.Hacc = vscale(this.Hacc, 0.987);
      const Hn = vnorm(this.Hacc);
      const minImp = minColImpulse(cols, isolated);
      const cooling = t - this.lastFireT < this.cfg.commandDelay + MIN_PULSE;
      if (!cooling && Hn >= 0.42 * minImp) {
        const alloc = allocateTorque(vscale(this.Hacc, 1 / MIN_PULSE), cols, isolated, {
          wantNGate: 0.02,
        });
        const use = alloc.ids.length === 0
          ? alignAllocate(this.Hacc, cols, isolated, MIN_PULSE, true)
          : alloc.pulse;
        const delivered = pulseImpulse(cols, use);
        const dN = vnorm(delivered);
        const align = dN > 1e-9 && Hn > 1e-9 ? vdot(delivered, this.Hacc) / (dN * Hn) : 0;
        if (dN > 1e-9 && align >= 0.22) {
          for (let i = 0; i < 6; i++) if (use[i]) pulse[i] = use[i]!;
          this.Hacc = vsub(this.Hacc, delivered);
          this.lastFireT = t;
        }
      }
      return { sliderForce: Fs, pulseWidth: pulse };
    }

    const acq = alignAllocate(tauDes, cols, isolated, CTRL_DT, false);
    for (let i = 0; i < 6; i++) pulse[i] = acq[i] ?? 0;
    if (probe.probe && this.fdir.detectedFailedThruster < 0 && !isolated.has(probe.probeId)) {
      if ((pulse[probe.probeId] ?? 0) === 0) {
        const on = [0, 1, 2, 3, 4, 5].filter((i) => pulse[i]! > 0);
        if (on.length >= 2) {
          on.sort((a, b) => pulse[a]! - pulse[b]!);
          pulse[on[0]!] = 0;
        }
        pulse[probe.probeId] = CTRL_DT;
      }
    }
    if (pulse.some((p) => p > 0)) this.lastFireT = t;
    const delivered = pulseImpulse(cols, pulse);
    this.Hacc = vadd(vscale(this.Hacc, 0.5), vsub(vscale(tauDes, CTRL_DT), delivered));
    return { sliderForce: Fs, pulseWidth: pulse };
  }
}

function minColImpulse(cols: Vec3[], isolated: Set<number>): number {
  let m = Infinity;
  for (let i = 0; i < cols.length; i++) {
    if (isolated.has(i)) continue;
    const n = vnorm(cols[i]!) * MIN_PULSE;
    if (n > 1e-9 && n < m) m = n;
  }
  return Number.isFinite(m) ? m : 0.6;
}

/** Alignment-based 1–2 jet allocator (same-duty pair). Saturates for detumble. */
function alignAllocate(
  tauDes: Vec3,
  cols: Vec3[],
  isolated: Set<number>,
  maxWidth: number,
  minOnly: boolean,
  preferSingle = false,
): [number, number, number, number, number, number] {
  const pulse: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
  const wantN = vnorm(tauDes);
  if (wantN < 0.15 && !minOnly) return pulse;

  type Cand = { ids: number[]; tau: Vec3; align: number };
  const cands: Cand[] = [];
  const push = (ids: number[], tau: Vec3) => {
    const n = vnorm(tau);
    if (n < 1e-6) return;
    cands.push({ ids, tau, align: vdot(tau, tauDes) / n });
  };
  for (let i = 0; i < 6; i++) {
    if (isolated.has(i)) continue;
    push([i], cols[i]!);
    if (minOnly || preferSingle) continue;
    for (let j = i + 1; j < 6; j++) {
      if (isolated.has(j)) continue;
      push([i, j], vadd(cols[i]!, cols[j]!));
    }
  }
  if (preferSingle && !minOnly) {
    for (let i = 0; i < 6; i++) {
      if (isolated.has(i)) continue;
      for (let j = i + 1; j < 6; j++) {
        if (isolated.has(j)) continue;
        push([i, j], vadd(cols[i]!, cols[j]!));
      }
    }
  }
  cands.sort((a, b) => {
    const bonus = (c: Cand) => (preferSingle && c.ids.length === 1 ? 0.15 : 0);
    return b.align + bonus(b) - (a.align + bonus(a));
  });
  const best = cands[0];
  if (!best || best.align < 0.02) return pulse;
  if (minOnly) {
    pulse[best.ids[0]!] = MIN_PULSE;
    return pulse;
  }
  const mag = vnorm(best.tau);
  let duty = clamp(wantN / mag, 0, 1);
  if (wantN < 8) duty = Math.min(duty, 0.7);
  if (duty * CTRL_DT < MIN_PULSE) {
    if (wantN > 0.5) duty = MIN_PULSE / CTRL_DT;
    else return pulse;
  }
  const width = Math.min(maxWidth, duty * CTRL_DT);
  for (const id of best.ids) pulse[id] = width;
  return pulse;
}
