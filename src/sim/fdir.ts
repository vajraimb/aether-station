/**
 * FDIR from delayed command-vs-current residuals. Never keys off a wall-clock
 * fault injection time and never assumes a particular thruster identity.
 */
import { clamp } from "./math3d";
import type { Command, Observation, PublicConfig } from "./types";

export interface FdirSnapshot {
  faultConfidence: [number, number, number, number, number, number];
  detectedFailedThruster: number;
  detectionTime: number | null;
  isolationTime: number | null;
  isolationConfidence: number;
  abnormalFlagTime: number | null;
}

export class FdirEngine {
  faultConfidence: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
  detectedFailedThruster = -1;
  detectionTime: number | null = null;
  isolationTime: number | null = null;
  isolationConfidence = 0;
  abnormalFlagTime: number | null = null;
  isolated = new Set<number>();
  private cmdHistory: { t: number; pulse: number[] }[] = [];
  private probeId = 0;
  private probeUntil = -1;
  readonly healthy = new Set<number>();

  pushCommand(t: number, cmd: Command) {
    this.cmdHistory.push({ t, pulse: [...cmd.pulseWidth] });
    if (this.cmdHistory.length > 48) this.cmdHistory.shift();
  }

  snapshot(): FdirSnapshot {
    return {
      faultConfidence: [...this.faultConfidence],
      detectedFailedThruster: this.detectedFailedThruster,
      detectionTime: this.detectionTime,
      isolationTime: this.isolationTime,
      isolationConfidence: this.isolationConfidence,
      abnormalFlagTime: this.abnormalFlagTime,
    };
  }

  update(obs: Observation, cfg: PublicConfig): { probeId: number; probe: boolean } {
    const t = obs.timestamp;
    const delay = cfg.commandDelay;
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

    if (obs.actuatorResponseAbnormal && this.abnormalFlagTime === null) {
      this.abnormalFlagTime = t;
    }

    for (let i = 0; i < 6; i++) {
      const curr = obs.thrusterCurrentFeedback[i] ?? 0;
      if (shouldOn[i] && curr < 0.22) {
        this.faultConfidence[i] = clamp(this.faultConfidence[i] + 0.40, 0, 1);
        this.healthy.delete(i);
      } else if (shouldOn[i] && curr > 0.5) {
        this.faultConfidence[i] = clamp(this.faultConfidence[i] - 0.22, 0, 1);
        this.healthy.add(i);
      } else if (!obs.actuatorResponseAbnormal) {
        this.faultConfidence[i] = clamp(this.faultConfidence[i] * 0.92, 0, 1);
      }
    }

    if (obs.actuatorResponseAbnormal && this.detectionTime === null) {
      this.detectionTime = t;
    }

    let probe = false;
    let probeId = this.probeId;
    if (this.detectedFailedThruster < 0) {
      let best = -1;
      let bestC = obs.actuatorResponseAbnormal ? 0.42 : 0.97;
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
      } else if (obs.actuatorResponseAbnormal && t >= this.probeUntil) {
        for (let k = 0; k < 6; k++) {
          const id = (this.probeId + k) % 6;
          if (this.isolated.has(id)) continue;
          probeId = id;
          this.probeId = (id + 1) % 6;
          this.probeUntil = t + 0.22;
          probe = true;
          break;
        }
      }
    }

    return { probeId, probe };
  }
}
