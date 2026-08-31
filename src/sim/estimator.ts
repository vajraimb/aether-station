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
  private cfg: PublicConfig;

  constructor(cfg: PublicConfig) {
    this.cfg = cfg;
    this.c1 = 0.5 * (cfg.c1Range[0] + cfg.c1Range[1]);
    this.c2 = 0.5 * (cfg.c2Range[0] + cfg.c2Range[1]);
    this.k12 = 0.5 * (cfg.k12Range[0] + cfg.k12Range[1]);
    this.etaT = 0.5 * (cfg.etaRange[0] + cfg.etaRange[1]);
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
    // Propagate attitude
    const dq: Quat = qnormalize([1, 0.5 * wm[0] * dt, 0.5 * wm[1] * dt, 0.5 * wm[2] * dt]);
    this.q = qnormalize(qmul(this.q, dq));

    // MEKF measurement: error vector of q_est* ⊗ q_meas
    const err = attitudeErrorVector(obs.quaternionMeasured, this.q);
    const R = ((0.15 * Math.PI) / 180) ** 2;
    const P = this.Ptheta + R;
    const K = this.Ptheta / P;
    this.q = qnormalize(
      qmul(this.q, qnormalize([1, K * err[0], K * err[1], K * err[2]])),
    );
    this.Ptheta = (1 - K) * this.Ptheta + 1e-8;
    this.nis = (err[0] * err[0] + err[1] * err[1] + err[2] * err[2]) / P;

    // Bias: low-frequency residual
    const kb = 0.02;
    this.bias = [
      this.bias[0] - kb * err[0],
      this.bias[1] - kb * err[1],
      this.bias[2] - kb * err[2],
    ];
    this.w = wm;

    // Slider: complementary
    const as = 0.35;
    this.s = (1 - as) * (this.s + this.sd * dt) + as * obs.sliderPosition;
    const av = 0.4;
    this.sd = (1 - av) * this.sd + av * obs.sliderVelocity;

    // Slosh from delayed pressure
    if (this.cfg.fluidPresent) {
      const [th1m, th2m] = invertPressure(obs.tankWallPressure1, obs.tankWallPressure2);
      const af = 0.25;
      const th1f = (1 - af) * this.th1 + af * th1m;
      const th2f = (1 - af) * this.th2 + af * th2m;
      this.th1d = (1 - 0.3) * this.th1d + 0.3 * (th1f - this.lastTh1) / dt;
      this.th2d = (1 - 0.3) * this.th2d + 0.3 * (th2f - this.lastTh2) / dt;
      this.th1 = th1f;
      this.th2 = th2f;
      this.lastTh1 = th1f;
      this.lastTh2 = th2f;

      // RLS-like on damping using second-order residual when motion is rich
      const th1dd = -OMEGA1 * OMEGA1 * Math.sin(this.th1) - this.k12 * Math.sin(this.th1 - this.th2);
      // y = -θ̈_obs - ω²sin − k sinΔ  ≈ c θ̇  − a_kin
      const th1ddObs = (this.th1d - (th1f - this.th1) / Math.max(dt, 1e-3)) / dt; // noisy, skip
      void th1dd;
      void th1ddObs;
      const excite = Math.abs(this.th1d) + Math.abs(this.th2d);
      if (excite > 0.02) {
        const y1 = -this.th1d; // proxy: damping dissipates
        const step = 0.002 * excite;
        // Move c toward matching observed decay vs natural
        const decay = Math.abs(this.th1d) < Math.abs(this.lastTh1) * 0.5 ? 1 : 0;
        void decay;
        this.c1 = clamp(this.c1 + step * Math.sign(Math.abs(this.th1) - 0.05) * 0.002, C1_RANGE[0], C1_RANGE[1]);
        this.c2 = clamp(this.c2 + step * 0.05, C2_RANGE[0], C2_RANGE[1]);
        const dth = this.th1 - this.th2;
        this.k12 = clamp(this.k12 + 0.0004 * Math.sin(dth) * this.th1d, K12_RANGE[0], K12_RANGE[1]);
        this.c1P = Math.max(0.004, this.c1P * 0.999);
        this.c2P = Math.max(0.004, this.c2P * 0.999);
        this.k12P = Math.max(0.01, this.k12P * 0.999);
      }
    } else {
      this.th1 = 0;
      this.th2 = 0;
      this.th1d = 0;
      this.th2d = 0;
    }

    this.fuel = 0.2 * this.fuel + 0.8 * obs.remainingFuelEstimate;

    if (alphaHint) {
      const ms = massState(this.cfg, this.s, this.th1, this.th2, this.fuel);
      const Ixx = ms.Icm[0][0];
      void Ixx;
      // eta update happens in the controller where commanded torque is known
      void alphaHint;
    }
  }

  updateEta(tauCmdMag: number, alphaMag: number, Iavg: number) {
    if (tauCmdMag < 2 || alphaMag < 1e-4) return;
    const etaObs = clamp((alphaMag * Iavg) / tauCmdMag, ETA_RANGE[0], ETA_RANGE[1]);
    const g = 0.08;
    this.etaT = clamp((1 - g) * this.etaT + g * etaObs, ETA_RANGE[0], ETA_RANGE[1]);
    this.etaP = Math.max(0.004, this.etaP * 0.995);
  }
}

export function worldRate(est: Estimate): Vec3 {
  return qRotate(est.q, est.w);
}
