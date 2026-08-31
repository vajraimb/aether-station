import {
  attitudeErrorAngle,
  attitudeErrorVector,
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
import { sliderForceCommand, torqueColumns } from "./allocate";
import { FdirEngine, type FdirSnapshot } from "./fdir";
import { CTRL_DT, MIN_PULSE } from "./constants";
import { FUEL_HARD, FUEL_STOP } from "./evalset";
import { choosePulse, pulseAlongWant, type PlannerOpts } from "./planner";
import type { Command, Observation, PublicConfig } from "./types";

/**
 * Observation-only GNC. Sees ONLY a serialised Observation. Never holds a
 * Simulator, never reads PrivateScenario, never keys off a wall-clock fault
 * time or a hard-coded thruster identity.
 *
 * Large-error law: receding-horizon bang-coast-bang on the short-way
 * eigenaxis (planner). Inside ~1.55° a finish pulse machine fires min-width
 * jets. Fuel floor is a constraint, not an objective.
 */
export class AgentController {
  readonly name: string = "observation";
  readonly estimator: Estimator;
  protected cfg: PublicConfig;
  readonly fdir = new FdirEngine();
  protected plannerOpts: PlannerOpts = { horizon: 8, wCap: 0.04, alphaScale: 0.5 };
  private lastCmd: Command = { sliderForce: 0, pulseWidth: [0, 0, 0, 0, 0, 0] };
  private lastW: Vec3 = [0, 0, 0];
  private lastObs: Observation | null = null;
  private Hacc: Vec3 = [0, 0, 0];
  private lastFireT = -1e9;
  private latched = false;
  private finishPulseT = -1e9;

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

  protected control(
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

    const needProbe =
      probe.probe && this.fdir.detectedFailedThruster < 0 && !isolated.has(probe.probeId);
    if (needProbe) pulse[probe.probeId] = CTRL_DT;

    const eN0mag = vnorm(attErr);
    const eN0: Vec3 = eN0mag > 1e-9 ? vscale(attErr, 1 / eN0mag) : [1, 0, 0];
    const wPar0 = vdot(w, eN0);

    if (attDeg < 0.92 && wmag < 0.0052 && wPar0 < 0.0012) this.latched = true;
    if (attDeg > 1.15 || wmag > 0.012 || wPar0 > 0.0035) this.latched = false;

    const tGo = Math.max(3, this.cfg.duration - t);
    const H = Math.min(this.plannerOpts.horizon ?? 8, tGo);
    const coastDeg = deg(
      vnorm([
        2 * attErr[0] + w[0] * H,
        2 * attErr[1] + w[1] * H,
        2 * attErr[2] + w[2] * H,
      ]),
    );

    if (
      ((this.latched && attDeg < 1.04 && wmag < 0.007 && wPar0 < 0.002) ||
        (attDeg < 0.90 && wmag < 0.0055 && coastDeg < 0.98 && wPar0 < 0.0012)) &&
      !needProbe
    ) {
      this.Hacc = vscale(this.Hacc, 0.75);
      return { sliderForce: Fs, pulseWidth: pulse };
    }

    const fuelPanic = est.fuel < FUEL_STOP;
    if (fuelPanic && !needProbe) {
      const lastChance =
        attDeg > 1.15 && attDeg < 8 && wmag < 0.035 && est.fuel > FUEL_HARD + 0.03;
      if (!lastChance) {
        this.Hacc = vscale(this.Hacc, 0.7);
        return { sliderForce: Fs, pulseWidth: pulse };
      }
      const cooling = t - this.lastFireT < this.cfg.commandDelay + 0.4 * MIN_PULSE;
      if (!cooling && wPar0 > -0.01 && attDeg > 1.15) {
        const use = pulseAlongWant(vscale(eN0, -1), cols, isolated, MIN_PULSE, 1);
        for (let i = 0; i < 6; i++) pulse[i] = use[i] ?? 0;
        if (pulse.some((p) => p > 0)) this.lastFireT = t;
      }
      return { sliderForce: Fs, pulseWidth: pulse };
    }

    const slew = attDeg > 1.55 || wmag > 0.04 || wPar0 > 0.008;
    if (slew) {
      const planned = choosePulse(this.cfg, est, isolated, t, {
        ...this.plannerOpts,
        lastFireT: this.lastFireT,
      });
      for (let i = 0; i < 6; i++) pulse[i] = planned[i] ?? 0;
      if (needProbe) {
        const on = [0, 1, 2, 3, 4, 5].filter((i) => (pulse[i] ?? 0) > 0);
        if (on.length >= 2) {
          on.sort((a, b) => (pulse[a] ?? 0) - (pulse[b] ?? 0));
          pulse[on[0]!] = 0;
        }
        pulse[probe.probeId] = CTRL_DT;
      }
      if (pulse.some((p) => p > 0)) this.lastFireT = t;
      return { sliderForce: Fs, pulseWidth: pulse };
    }

    const eNmag = vnorm(attErr);
    const eN: Vec3 = eNmag > 1e-9 ? vscale(attErr, 1 / eNmag) : [1, 0, 0];
    const wParNow = vdot(w, eN);
    const wPerp = vsub(w, vscale(eN, wParNow));
    const wPerpN = vnorm(wPerp);
    const cooling = t - this.lastFireT < this.cfg.commandDelay + 0.4 * MIN_PULSE;

    if (!cooling) {
      if (wParNow > 0.0014 && attDeg > 0.7) {
        const width = wParNow > 0.004 ? 2 * MIN_PULSE : MIN_PULSE;
        const use = pulseAlongWant(vscale(eN, -1), cols, isolated, width, 1);
        for (let i = 0; i < 6; i++) pulse[i] = use[i] ?? 0;
        if (pulse.some((p) => p > 0)) this.lastFireT = t;
      } else if (wPerpN > 0.006 && attDeg > 0.7) {
        const use = pulseAlongWant(vscale(wPerp, -1), cols, isolated, MIN_PULSE, 1);
        for (let i = 0; i < 6; i++) pulse[i] = use[i] ?? 0;
        if (pulse.some((p) => p > 0)) this.lastFireT = t;
      } else if (wParNow < -0.009 && attDeg < 1.4) {
        const use = pulseAlongWant(eN, cols, isolated, MIN_PULSE, 1);
        for (let i = 0; i < 6; i++) pulse[i] = use[i] ?? 0;
        if (pulse.some((p) => p > 0)) this.lastFireT = t;
      } else if (
        attDeg > 0.95 &&
        wmag <= 0.003 &&
        t - this.finishPulseT > 1.6 &&
        est.fuel >= FUEL_STOP - 0.04
      ) {
        const use = pulseAlongWant(vscale(eN, -1), cols, isolated, MIN_PULSE, 1);
        for (let i = 0; i < 6; i++) pulse[i] = use[i] ?? 0;
        if (pulse.some((p) => p > 0)) {
          this.lastFireT = t;
          this.finishPulseT = t;
        }
      }
    }
    if (needProbe) {
      const on = [0, 1, 2, 3, 4, 5].filter((i) => (pulse[i] ?? 0) > 0);
      if (on.length >= 2) {
        on.sort((a, b) => (pulse[a] ?? 0) - (pulse[b] ?? 0));
        pulse[on[0]!] = 0;
      }
      pulse[probe.probeId] = CTRL_DT;
    }
    return { sliderForce: Fs, pulseWidth: pulse };
  }
}
