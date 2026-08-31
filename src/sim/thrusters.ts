import { vadd, vcross, vscale, type Vec3 } from "./math3d";
import { CMD_DELAY, FMAX, MAX_ACTIVE, MIN_PULSE, THRUSTERS } from "./constants";
import type { Command, ThrusterGeom } from "./types";

export { THRUSTERS };

export interface ThrusterSnapshot {
  commanded: [number, number, number, number, number, number];
  actual: [number, number, number, number, number, number];
  current: [number, number, number, number, number, number];
  Fb: Vec3;
  tauO: Vec3;
  nActive: number;
  fuelDot: number;
}

type Pulse = { id: number; tOn: number; tOff: number };

export class ThrusterSystem {
  private queue: Pulse[] = [];
  private failed = new Set<number>();
  private eta: number;
  private Fmax: number;
  private delay: number;
  private minPulse: number;
  private maxActive: number;
  private ispG0: number;
  lastCmd: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
  pulseCount = 0;
  totalOnTime = 0;

  constructor(eta: number, opts?: { Fmax?: number; delay?: number; minPulse?: number; isp?: number; g0?: number }) {
    this.eta = eta;
    this.Fmax = opts?.Fmax ?? FMAX;
    this.delay = opts?.delay ?? CMD_DELAY;
    this.minPulse = opts?.minPulse ?? MIN_PULSE;
    this.maxActive = MAX_ACTIVE;
    this.ispG0 = (opts?.isp ?? 68) * (opts?.g0 ?? 9.80665);
  }

  fail(id: number) {
    this.failed.add(id);
  }

  isFailed(id: number) {
    return this.failed.has(id);
  }

  submit(t: number, cmd: Command) {
    const widths = [...cmd.pulseWidth] as number[];
    this.lastCmd = [0, 0, 0, 0, 0, 0];
    const candidates: { id: number; w: number }[] = [];
    for (let i = 0; i < 6; i++) {
      let w = widths[i] ?? 0;
      if (w > 0 && w < this.minPulse) w = this.minPulse;
      if (w <= 0) continue;
      candidates.push({ id: i, w });
    }
    candidates.sort((a, b) => b.w - a.w);
    for (let k = 0; k < Math.min(this.maxActive, candidates.length); k++) {
      const c = candidates[k]!;
      this.lastCmd[c.id] = 1;
      this.queue.push({ id: c.id, tOn: t + this.delay, tOff: t + this.delay + c.w });
      this.pulseCount += 1;
    }
  }

  evaluate(t: number, dt: number, fuel: number): ThrusterSnapshot {
    this.queue = this.queue.filter((p) => p.tOff > t);
    const live = this.queue.filter((p) => t >= p.tOn && t < p.tOff);
    live.sort((a, b) => b.tOn - a.tOn);
    const chosen: Pulse[] = [];
    const seen = new Set<number>();
    for (const p of live) {
      if (seen.has(p.id)) continue;
      if (chosen.length >= this.maxActive && !this.failed.has(p.id)) continue;
      const thrusting = chosen.filter((q) => !this.failed.has(q.id)).length;
      if (!this.failed.has(p.id) && thrusting >= this.maxActive) continue;
      chosen.push(p);
      seen.add(p.id);
    }
    const commanded: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
    const actual: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
    const current: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
    let Fb: Vec3 = [0, 0, 0];
    let tauO: Vec3 = [0, 0, 0];
    let nActive = 0;
    const dry = fuel <= 1e-6;
    for (const p of live) commanded[p.id] = 1;
    for (const p of chosen) {
      const failed = this.failed.has(p.id);
      if (!failed && !dry) {
        nActive += 1;
        actual[p.id] = 1;
        current[p.id] = 1;
        const g = THRUSTERS[p.id] as ThrusterGeom;
        const Fi = vscale(g.dir, this.eta * this.Fmax);
        Fb = vadd(Fb, Fi);
        tauO = vadd(tauO, vcross(g.pos, Fi));
        this.totalOnTime += dt;
      } else if (failed) {
        current[p.id] = 0.04;
      } else if (dry) {
        current[p.id] = 0.02;
      }
    }
    let sumAbs = 0;
    for (let i = 0; i < 6; i++) if (actual[i]) sumAbs += this.eta * this.Fmax;
    const fuelDot = dry ? 0 : -sumAbs / this.ispG0;
    return { commanded, actual, current, Fb, tauO, nActive, fuelDot };
  }
}

export function torqueColumns(rCmB: Vec3, eta: number, Fmax = FMAX): Vec3[] {
  return THRUSTERS.map((g) => {
    const Fi = vscale(g.dir, eta * Fmax);
    const r = vsubPos(g.pos, rCmB);
    return vcross(r, Fi);
  });
}

function vsubPos(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}
