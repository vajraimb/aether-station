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
  type Vec3,
} from "./math3d";
import { massState } from "./dynamics";
import { Estimator, type Estimate } from "./estimator";
import { THRUSTERS } from "./thrusters";
import { CTRL_DT, MIN_PULSE } from "./constants";
import type { Command, Observation, PublicConfig } from "./types";

/**
 * Hierarchical agent. Sees ONLY Observation. Never reads private scenario,
 * never holds a Simulator, never keys off absolute time for the fault id.
 */
export class AgentController {
  readonly estimator: Estimator;
  private cfg: PublicConfig;
  private isolated = new Set<number>();
  faultConfidence: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
  detectedFailedThruster = -1;
  detectionTime: number | null = null;
  isolationTime: number | null = null;
  isolationConfidence = 0;
  private lastCmd: Command = {
    sliderForce: 0,
    pulseWidth: [0, 0, 0, 0, 0, 0],
  };
  private cmdHistory: { t: number; pulse: number[] }[] = [];
  private lastObs: Observation | null = null;
  private lastW: Vec3 = [0, 0, 0];
  private lastEstT = 0;
  private probeId = 0;
  private probeUntil = 0;
  private settledSince: number | null = null;
  private sawCurrent = false;
  private holdMode = false;

  constructor(cfg: PublicConfig) {
    this.cfg = cfg;
    this.estimator = new Estimator(cfg);
  }

  get estimate(): Estimate {
    return this.estimator.snapshot();
  }

  step(obs: Observation): Command {
    this.estimator.update(obs, null);
    const est = this.estimator.snapshot();
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
    this.fdir(obs, est, alpha);
    this.lastObs = obs;
    this.lastEstT = obs.timestamp;

    const cmd = this.control(obs.timestamp, est);
    this.lastCmd = cmd;
    this.cmdHistory.push({ t: obs.timestamp, pulse: [...cmd.pulseWidth] });
    if (this.cmdHistory.length > 40) this.cmdHistory.shift();
    return cmd;
  }

  private fdir(obs: Observation, est: Estimate, alpha: Vec3) {
    const t = obs.timestamp;
    const delay = this.cfg.commandDelay;
    const shouldOn = [false, false, false, false, false, false];
    for (const c of this.cmdHistory) {
      for (let i = 0; i < 6; i++) {
        const w = c.pulse[i] ?? 0;
        if (w <= 0) continue;
        const tOn = c.t + delay;
        const tOff = tOn + w;
        if (t >= tOn && t < tOff) shouldOn[i] = true;
      }
    }
    for (let i = 0; i < 6; i++) {
      const curr = obs.thrusterCurrentFeedback[i] ?? 0;
      if (shouldOn[i] && curr < 0.22) {
        this.faultConfidence[i] = clamp(this.faultConfidence[i] + 0.28, 0, 1);
      } else if (shouldOn[i] && curr > 0.5) {
        this.faultConfidence[i] = clamp(this.faultConfidence[i] - 0.18, 0, 1);
      } else if (!obs.actuatorResponseAbnormal) {
        this.faultConfidence[i] = clamp(this.faultConfidence[i] * 0.92, 0, 1);
      }
    }

    if (obs.actuatorResponseAbnormal && this.detectionTime === null) {
      this.detectionTime = t;
    }

    if (this.detectedFailedThruster < 0) {
      let best = -1;
      let bestC = obs.actuatorResponseAbnormal ? 0.45 : 0.97;
      for (let i = 0; i < 6; i++) {
        if (this.faultConfidence[i] > bestC) {
          bestC = this.faultConfidence[i];
          best = i;
        }
      }
      if (best >= 0) {
        this.detectedFailedThruster = best;
        this.isolationTime = t;
        this.isolationConfidence = bestC;
        this.isolated.add(best);
        if (this.detectionTime === null) this.detectionTime = t;
      } else if (obs.actuatorResponseAbnormal && t > this.probeUntil + 0.25) {
        this.probeId = (this.probeId + 1) % 6;
        this.probeUntil = t + 0.35;
      }
    }

    const tauCmd = this.predictedTorque(this.lastCmd.pulseWidth, est, 1);
    this.estimator.updateEta(vnorm(tauCmd), vnorm(alpha), 620);
  }

  private predictedTorque(pulse: number[], est: Estimate, eta: number): Vec3 {
    const ms = massState(this.cfg, est.s, est.th1, est.th2, est.fuel);
    let tau: Vec3 = [0, 0, 0];
    for (let i = 0; i < 6; i++) {
      const w = pulse[i] ?? 0;
      if (w <= 0) continue;
      const duty = clamp(w / CTRL_DT, 0, 1);
      const g = THRUSTERS[i]!;
      const Fi = vscale(g.dir, eta * this.cfg.maxThrust * duty);
      const r: Vec3 = [g.pos[0] - ms.rCmB[0], g.pos[1] - ms.rCmB[1], g.pos[2] - ms.rCmB[2]];
      tau = vadd(tau, [
        r[1] * Fi[2] - r[2] * Fi[1],
        r[2] * Fi[0] - r[0] * Fi[2],
        r[0] * Fi[1] - r[1] * Fi[0],
      ]);
    }
    return tau;
  }

  private control(t: number, est: Estimate): Command {
    const pulse: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];

    const s = est.s;
    const sd = est.sd;
    const wn = 1.05;
    const z = 1.15;
    const Kp = this.cfg.sliderMass * wn * wn;
    const Kd = 2 * z * wn * this.cfg.sliderMass;
    let Fs = -Kp * (s - this.cfg.sTarget) - Kd * sd;
    const margin = this.cfg.sliderMax - Math.abs(s);
    if (margin < 0.35 && s * sd > 0) {
      Fs = -Math.sign(sd) * this.cfg.sliderForceMax;
    }
    if (Math.abs(s) > 1.55 && Math.abs(sd) > 0.08) {
      Fs = -Math.sign(sd) * this.cfg.sliderForceMax;
    }
    Fs = clamp(Fs, -this.cfg.sliderForceMax, this.cfg.sliderForceMax);

    const q = qnormalize(est.q);
    const w = est.w;
    const attErr = attitudeErrorVector(q, this.cfg.qTarget);
    const attDeg = deg(attitudeErrorAngle(q, this.cfg.qTarget));
    const wmag = vnorm(w);

    const rateOnly = wmag > 0.08;
    let tauDes: Vec3;
    if (rateOnly) {
      tauDes = [
        -200 * w[0] - 12 * 2 * attErr[0],
        -200 * w[1] - 12 * 2 * attErr[1],
        -200 * w[2] - 12 * 2 * attErr[2],
      ];
    } else {
      const terminal = attDeg < 4;
      const slow = wmag < 0.02;
      let KpA: number;
      let KdA: number;
      if (terminal) {
        KpA = attDeg < 2 ? 8 : 22;
        KdA = 170;
      } else if (slow) {
        KpA = attDeg > 6 ? 48 : 36;
        KdA = 55;
      } else {
        KpA = attDeg > 25 ? 28 : 18;
        KdA = 95;
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

    if (attDeg < 0.55) this.holdMode = true;
    if (this.holdMode) {
      if (wmag < 0.004) return { sliderForce: Fs, pulseWidth: pulse };
      tauDes = vscale(w, -280);
    }

    if (
      this.detectedFailedThruster < 0 &&
      this.lastObs?.actuatorResponseAbnormal &&
      t <= this.probeUntil &&
      t > this.probeUntil - 0.45
    ) {
      const id = this.probeId;
      if (!this.isolated.has(id)) pulse[id] = CTRL_DT;
      // Keep the rest of attitude allocation too; evaluate() will cap at 2.
    }

    this.allocate(tauDes, est, pulse);
    return { sliderForce: Fs, pulseWidth: pulse };
  }

  private allocate(
    tauDes: Vec3,
    est: Estimate,
    pulse: [number, number, number, number, number, number],
  ) {
    const ms = massState(this.cfg, est.s, est.th1, est.th2, est.fuel);
    const eta = this.estimator.etaT;
    const cols: Vec3[] = THRUSTERS.map((g) => {
      const Fi = vscale(g.dir, eta * this.cfg.maxThrust);
      const r: Vec3 = [g.pos[0] - ms.rCmB[0], g.pos[1] - ms.rCmB[1], g.pos[2] - ms.rCmB[2]];
      return [
        r[1] * Fi[2] - r[2] * Fi[1],
        r[2] * Fi[0] - r[0] * Fi[2],
        r[0] * Fi[1] - r[1] * Fi[0],
      ];
    });
    const wantN = vnorm(tauDes);
    if (wantN < 0.2) return;

    type Cand = { ids: number[]; tau: Vec3; align: number };
    const cands: Cand[] = [];
    const push = (ids: number[], tau: Vec3) => {
      const n = vnorm(tau);
      if (n < 1e-6) return;
      const align = vdot(tau, tauDes) / n;
      cands.push({ ids, tau, align });
    };
    for (let i = 0; i < 6; i++) {
      if (this.isolated.has(i)) continue;
      push([i], cols[i]!);
      for (let j = i + 1; j < 6; j++) {
        if (this.isolated.has(j)) continue;
        push(
          [i, j],
          [cols[i]![0] + cols[j]![0], cols[i]![1] + cols[j]![1], cols[i]![2] + cols[j]![2]],
        );
      }
    }
    const attNow = deg(attitudeErrorAngle(est.q, this.cfg.qTarget));
    cands.sort((a, b) => {
      const bonus = (c: Cand) => (attNow < 6 && c.ids.length === 1 ? 0.12 : 0);
      return b.align + bonus(b) - (a.align + bonus(a));
    });
    const best = cands[0];
    const minAlign = 0.02;
    if (!best || best.align < minAlign) return;
    const mag = vnorm(best.tau);
    let duty = clamp(wantN / mag, 0, 1);
    if (wantN < 8) duty = Math.min(duty, 0.7);
    if (est.fuel < 3.0 && wantN < 12) duty = Math.min(duty, 0.6);
    if (duty * CTRL_DT < MIN_PULSE) {
      if (wantN > 0.8) duty = MIN_PULSE / CTRL_DT;
      else return;
    }
    for (const id of best.ids) pulse[id] = Math.min(CTRL_DT, duty * CTRL_DT);
  }
}
