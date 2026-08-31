import {
  attitudeErrorVector,
  clamp,
  qmul,
  qnormalize,
  qRotate,
  type Quat,
  type Vec3,
} from "./math3d";
import { invertPressure, massState } from "./dynamics";
import {
  C1_RANGE,
  C2_RANGE,
  ETA_RANGE,
  K12_RANGE,
  OMEGA1,
  OMEGA2,
} from "./constants";
import type { Observation, PublicConfig } from "./types";

export interface Estimate {
  q: Quat;
  w: Vec3;
  bias: Vec3;
  s: number;
  sd: number;
  th1: number;
  th1d: number;
  th2: number;
  th2d: number;
  fuel: number;
  c1: number;
  c2: number;
  k12: number;
  etaT: number;
  c1P: number;
  c2P: number;
  k12P: number;
  etaP: number;
  nis: number;
}

export class Estimator {
  q: Quat = [1, 0, 0, 0];
  bias: Vec3 = [0, 0, 0];
  w: Vec3 = [0, 0, 0];
  s = 0;
  sd = 0;
  th1 = 0;
  th1d = 0;
  th2 = 0;
  th2d = 0;
  fuel = 5;
  c1: number;
  c2: number;
  k12: number;
  etaT: number;
  c1P = 0.02;
  c2P = 0.02;
  k12P = 0.04;
  etaP = 0.04;
  Ptheta = 0.02;
  Pbias: Vec3 = [1e-5, 1e-5, 1e-5];
  nis = 0;
  private lastT: number | null = null;
  private lastTh1 = 0;
  private lastTh2 = 0;
  private lastTh1d = 0;
  private lastTh2d = 0;
  private cfg: PublicConfig;
  /** 2-param RLS for [c, k12] on each slosh mode. */
  private P1: number[][] = [
    [0.05, 0],
    [0, 0.1],
  ];
  private P2: number[][] = [
    [0.05, 0],
    [0, 0.1],
  ];
  private k12a = 0;
  private k12b = 0;
  private nFit = 0;
  private peak1 = 0;
  private peak1T = 0;
  private peak2 = 0;
  private peak2T = 0;
  private lastAmp1 = 0;
  private lastAmp2 = 0;

  constructor(cfg: PublicConfig) {
    this.cfg = cfg;
    this.c1 = 0.5 * (cfg.c1Range[0] + cfg.c1Range[1]);
    this.c2 = 0.5 * (cfg.c2Range[0] + cfg.c2Range[1]);
    this.k12 = 0.5 * (cfg.k12Range[0] + cfg.k12Range[1]);
    this.etaT = 0.5 * (cfg.etaRange[0] + cfg.etaRange[1]);
    this.k12a = this.k12;
    this.k12b = this.k12;
    this.fuel = cfg.initialFuelMass;
  }

  snapshot(): Estimate {
    return {
      q: [this.q[0], this.q[1], this.q[2], this.q[3]],
      w: [this.w[0], this.w[1], this.w[2]],
      bias: [this.bias[0], this.bias[1], this.bias[2]],
      s: this.s,
      sd: this.sd,
      th1: this.th1,
      th1d: this.th1d,
      th2: this.th2,
      th2d: this.th2d,
      fuel: this.fuel,
      c1: this.c1,
      c2: this.c2,
      k12: this.k12,
      etaT: this.etaT,
      c1P: this.c1P,
      c2P: this.c2P,
      k12P: this.k12P,
      etaP: this.etaP,
      nis: this.nis,
    };
  }

  update(obs: Observation, alphaHint: Vec3 | null) {
    const t = obs.timestamp;
    const dt = this.lastT === null ? this.cfg.sensorPeriod : Math.max(1e-4, t - this.lastT);
    this.lastT = t;

    const wm: Vec3 = [
      obs.gyroMeasured[0] - this.bias[0],
      obs.gyroMeasured[1] - this.bias[1],
      obs.gyroMeasured[2] - this.bias[2],
    ];
    const dq: Quat = qnormalize([1, 0.5 * wm[0] * dt, 0.5 * wm[1] * dt, 0.5 * wm[2] * dt]);
    this.q = qnormalize(qmul(this.q, dq));

    const err = attitudeErrorVector(obs.quaternionMeasured, this.q);
    const R = ((0.15 * Math.PI) / 180) ** 2;
    const P = this.Ptheta + R;
    const K = this.Ptheta / P;
    this.q = qnormalize(
      qmul(this.q, qnormalize([1, K * err[0], K * err[1], K * err[2]])),
    );
    this.Ptheta = (1 - K) * this.Ptheta + 4e-8;
    this.nis = (err[0] * err[0] + err[1] * err[1] + err[2] * err[2]) / P;

    const kb = 0.02;
    this.bias = [
      this.bias[0] - kb * err[0],
      this.bias[1] - kb * err[1],
      this.bias[2] - kb * err[2],
    ];
    this.w = wm;

    const as = 0.35;
    this.s = (1 - as) * (this.s + this.sd * dt) + as * obs.sliderPosition;
    const av = 0.4;
    this.sd = (1 - av) * this.sd + av * obs.sliderVelocity;

    if (this.cfg.fluidPresent) {
      const [th1m, th2m] = invertPressure(obs.tankWallPressure1, obs.tankWallPressure2);
      const af = 0.28;
      const th1f = (1 - af) * this.th1 + af * th1m;
      const th2f = (1 - af) * this.th2 + af * th2m;
      const th1dObs = (th1f - this.lastTh1) / dt;
      const th2dObs = (th2f - this.lastTh2) / dt;
      this.th1d = 0.65 * this.th1d + 0.35 * th1dObs;
      this.th2d = 0.65 * this.th2d + 0.35 * th2dObs;
      const th1dd = (this.th1d - this.lastTh1d) / dt;
      const th2dd = (this.th2d - this.lastTh2d) / dt;
      this.lastTh1d = this.th1d;
      this.lastTh2d = this.th2d;
      const prev1 = this.lastTh1;
      const prev2 = this.lastTh2;
      this.th1 = th1f;
      this.th2 = th2f;
      this.lastTh1 = th1f;
      this.lastTh2 = th2f;

      const quiet = Math.hypot(this.w[0], this.w[1], this.w[2]) < 0.07;
      const ok1 = Number.isFinite(th1dd) && Math.abs(th1dd) < 4 && Math.abs(this.th1d) > 0.03;
      const ok2 = Number.isFinite(th2dd) && Math.abs(th2dd) < 5 && Math.abs(this.th2d) > 0.03;
      if (quiet && ok1) {
        const y1 = -th1dd - OMEGA1 * OMEGA1 * Math.sin(this.th1);
        this.rlsStep(this.P1, [this.th1d, Math.sin(this.th1 - this.th2)], y1, (x) => {
          this.c1 = clamp(0.7 * this.c1 + 0.3 * x[0]!, C1_RANGE[0], C1_RANGE[1]);
          this.k12a = clamp(0.7 * this.k12a + 0.3 * x[1]!, K12_RANGE[0], K12_RANGE[1]);
        }, this.c1, this.k12a);
        this.nFit += 1;
      }
      if (quiet && ok2) {
        const y2 = -th2dd - OMEGA2 * OMEGA2 * Math.sin(this.th2);
        this.rlsStep(this.P2, [this.th2d, Math.sin(this.th2 - this.th1)], y2, (x) => {
          this.c2 = clamp(0.7 * this.c2 + 0.3 * x[0]!, C2_RANGE[0], C2_RANGE[1]);
          this.k12b = clamp(0.7 * this.k12b + 0.3 * x[1]!, K12_RANGE[0], K12_RANGE[1]);
        }, this.c2, this.k12b);
      }
      if (this.nFit > 8) {
        this.k12 = clamp(0.5 * (this.k12a + this.k12b), K12_RANGE[0], K12_RANGE[1]);
        this.c1P = Math.max(0.004, this.c1P * 0.997);
        this.c2P = Math.max(0.004, this.c2P * 0.997);
        this.k12P = Math.max(0.008, this.k12P * 0.997);
      }

      // Envelope decay: A(t) ~ exp(-c t / 2) for a linear oscillator.
      const a1 = Math.hypot(this.th1, this.th1d / OMEGA1);
      const a2 = Math.hypot(this.th2, this.th2d / OMEGA2);
      if (a1 > this.peak1 && a1 > 0.04) {
        this.peak1 = a1;
        this.peak1T = t;
      }
      if (a2 > this.peak2 && a2 > 0.03) {
        this.peak2 = a2;
        this.peak2T = t;
      }
      if (this.peak1 > 0.05 && t - this.peak1T > 1.2 && a1 > 0.02 && a1 < this.peak1) {
        const cEnv = clamp((-2 / Math.max(0.4, t - this.peak1T)) * Math.log(a1 / this.peak1), C1_RANGE[0], C1_RANGE[1]);
        this.c1 = clamp(0.94 * this.c1 + 0.06 * cEnv, C1_RANGE[0], C1_RANGE[1]);
      }
      if (this.peak2 > 0.04 && t - this.peak2T > 0.8 && a2 > 0.015 && a2 < this.peak2) {
        const cEnv = clamp((-2 / Math.max(0.4, t - this.peak2T)) * Math.log(a2 / this.peak2), C2_RANGE[0], C2_RANGE[1]);
        this.c2 = clamp(0.94 * this.c2 + 0.06 * cEnv, C2_RANGE[0], C2_RANGE[1]);
      }
      this.lastAmp1 = a1;
      this.lastAmp2 = a2;
      void prev1;
      void prev2;
    } else {
      this.th1 = 0;
      this.th2 = 0;
      this.th1d = 0;
      this.th2d = 0;
    }

    this.fuel = 0.25 * this.fuel + 0.75 * obs.remainingFuelEstimate;

    // Light shrink toward range midpoints so a noisy RLS cannot park on a bound.
    // 0.99992^3600 ≈ 0.75 — keep most of the RLS once it has data.
    const midC1 = 0.5 * (C1_RANGE[0] + C1_RANGE[1]);
    const midC2 = 0.5 * (C2_RANGE[0] + C2_RANGE[1]);
    const midK = 0.5 * (K12_RANGE[0] + K12_RANGE[1]);
    const midE = 0.5 * (ETA_RANGE[0] + ETA_RANGE[1]);
    if (this.nFit < 12) {
      this.c1 = clamp(0.9997 * this.c1 + 0.0003 * midC1, C1_RANGE[0], C1_RANGE[1]);
      this.c2 = clamp(0.9997 * this.c2 + 0.0003 * midC2, C2_RANGE[0], C2_RANGE[1]);
      this.k12 = clamp(0.9998 * this.k12 + 0.0002 * midK, K12_RANGE[0], K12_RANGE[1]);
    }
    this.etaT = clamp(0.9999 * this.etaT + 0.0001 * midE, ETA_RANGE[0], ETA_RANGE[1]);

    if (alphaHint) void alphaHint;
  }

  private rlsStep(
    P: number[][],
    phi: number[],
    y: number,
    apply: (x: number[]) => void,
    x0: number,
    x1: number,
  ) {
    if (!Number.isFinite(y) || Math.abs(y) > 40) return;
    const nphi = Math.hypot(phi[0]!, phi[1]!);
    if (nphi < 0.02) return;
    const lam = 0.993;
    const pphi = [
      P[0]![0]! * phi[0]! + P[0]![1]! * phi[1]!,
      P[1]![0]! * phi[0]! + P[1]![1]! * phi[1]!,
    ];
    const denom = lam + phi[0]! * pphi[0]! + phi[1]! * pphi[1]!;
    if (Math.abs(denom) < 1e-9) return;
    const g = [pphi[0]! / denom, pphi[1]! / denom];
    const x = [x0, x1];
    const e = y - (phi[0]! * x[0]! + phi[1]! * x[1]!);
    if (!Number.isFinite(e) || Math.abs(e) > 20) return;
    x[0]! += g[0]! * e;
    x[1]! += g[1]! * e;
    const Pnew = [
      [P[0]![0]! - g[0]! * pphi[0]!, P[0]![1]! - g[0]! * pphi[1]!],
      [P[1]![0]! - g[1]! * pphi[0]!, P[1]![1]! - g[1]! * pphi[1]!],
    ];
    P[0]![0] = Pnew[0]![0]! / lam;
    P[0]![1] = Pnew[0]![1]! / lam;
    P[1]![0] = Pnew[1]![0]! / lam;
    P[1]![1] = Pnew[1]![1]! / lam;
    apply(x);
  }

  updateEta(tauCmdMag: number, alphaMag: number, Iavg: number) {
    if (tauCmdMag < 1.5 || alphaMag < 5e-5) return;
    // columns already include etaT, so (α I)/τ_cmd ≈ eta_true / eta_est.
    const ratio = (alphaMag * Iavg) / tauCmdMag;
    const etaObs = clamp(this.etaT * ratio, ETA_RANGE[0], ETA_RANGE[1]);
    const g = 0.06;
    this.etaT = clamp((1 - g) * this.etaT + g * etaObs, ETA_RANGE[0], ETA_RANGE[1]);
    this.etaP = Math.max(0.003, this.etaP * 0.99);
  }
}

export function worldRate(est: Estimate): Vec3 {
  return qRotate(est.q, est.w);
}

void massState;
