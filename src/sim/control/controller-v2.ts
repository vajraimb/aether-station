/**
 * Observation-only discrete-pulse V2 controller.
 * Belief = estimator + FDIR. Planner never sees truth, scenario, or wall-clock faults.
 */
import { sliderForceCommand, torqueColumns } from "../allocate";
import { CTRL_DT, MIN_PULSE, THRUSTERS } from "../constants";
import { Estimator, type Estimate } from "../estimator";
import { FdirEngine, type FdirSnapshot } from "../fdir";
import { massState } from "../dynamics";
import {
  attitudeErrorAngle,
  deg,
  qnormalize,
  vnorm,
  type Vec3,
} from "../math3d";
import type { AnyController } from "../oracle";
import type { Command, Observation, PublicConfig } from "../types";
import { DEFAULT_BEAM_CONFIG, planBeam, type BeamDiagnostics, type BeamPlannerConfig } from "./beam-planner";
import { controlFromCommand } from "./convert";
import {
  commandFromPrimitive,
  generatePulsePrimitives,
  netWrenchForPrimitive,
  type PendingPulse,
  type PulseDurationS,
  type PulsePrimitive,
} from "./discrete-actions";
import type { PlantFlightController } from "./factory";
import type {
  ControlCommand,
  ControllerDiagnostics,
  FlightController,
  PublicControllerConfig,
} from "./interface";
import { rolloutFromSimLike, type RolloutParameters, type RolloutState } from "./rollout-model";

const DETUMBLE_SAFE = 0.12;

export class DiscretePulseV2Controller implements FlightController, PlantFlightController {
  readonly name = "discrete-pulse-v2";
  readonly estimator: Estimator;
  readonly fdir = new FdirEngine();
  private plant: PublicConfig;
  private flightCfg: PublicControllerConfig;
  private lastObs: Observation | null = null;
  private lastW: Vec3 = [0, 0, 0];
  private lastCmd: Command = { sliderForce: 0, pulseWidth: [0, 0, 0, 0, 0, 0] };
  private committedUntil = -1e9;
  private lastPlan: PulsePrimitive | null = null;
  private lastDiag: BeamDiagnostics | null = null;
  private replanCount = 0;
  private fallbackCount = 0;
  private expandedNodes = 0;
  private pendingPlant: PendingPulse[] = [];
  private lastReplanT = -1e9;

  constructor(plant: PublicConfig, config: Readonly<PublicControllerConfig>) {
    this.plant = plant;
    this.flightCfg = { ...config, mode: "discrete-pulse-v2" };
    this.estimator = new Estimator(plant);
  }

  reset(config: Readonly<PublicControllerConfig>): void {
    this.flightCfg = { ...config, mode: "discrete-pulse-v2" };
    this.lastPlan = null;
    this.committedUntil = -1e9;
    this.lastReplanT = -1e9;
  }

  step(observation: Readonly<Observation>): ControlCommand {
    return controlFromCommand(this.stepPlant(observation));
  }

  stepPlant(obs: Observation): Command {
    this.estimator.update(obs, null);
    const est = this.estimator.snapshot();
    const dt = Math.max(1e-3, obs.timestamp - (this.lastObs?.timestamp ?? obs.timestamp - CTRL_DT));
    const alpha: Vec3 = [
      (est.w[0] - this.lastW[0]) / dt,
      (est.w[1] - this.lastW[1]) / dt,
      (est.w[2] - this.lastW[2]) / dt,
    ];
    this.lastW = [est.w[0], est.w[1], est.w[2]];
    const probe = this.fdir.update(obs, this.plant);
    this.lastObs = obs;

    const Fs = sliderForceCommand(est.s, est.sd, this.plant);
    const pulse: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
    const isolated = this.fdir.isolated;
    const needProbe = probe.probe && this.fdir.detectedFailedThruster < 0 && !isolated.has(probe.probeId);

    const holding = obs.timestamp + 1e-9 < Math.max(this.committedUntil, this.lastReplanT + this.flightCfg.replanPeriodS);
    if (!holding) {
      const planned = this.replan(obs.timestamp, est);
      this.lastPlan = planned;
      if (planned.thrusterIds.length > 0) {
        for (const id of planned.thrusterIds) pulse[id] = planned.durationS;
        this.committedUntil = obs.timestamp + planned.durationS;
        this.pendingPlant.push(
          ...planned.thrusterIds.map((id) => ({
            id,
            tOn: obs.timestamp + this.plant.commandDelay,
            tOff: obs.timestamp + this.plant.commandDelay + planned.durationS,
          })),
        );
      }
    }

    if (needProbe) {
      const on = [0, 1, 2, 3, 4, 5].filter((i) => (pulse[i] ?? 0) > 0);
      if (on.length >= 2) {
        on.sort((a, b) => (pulse[a] ?? 0) - (pulse[b] ?? 0));
        pulse[on[0]!] = 0;
      }
      if (!isolated.has(probe.probeId)) pulse[probe.probeId] = CTRL_DT;
    }

    const cmd: Command = { sliderForce: Fs, pulseWidth: pulse };
    this.lastCmd = cmd;
    this.fdir.pushCommand(obs.timestamp, cmd);
    this.estimator.updateEta(vnorm(this.predictedTorque(cmd.pulseWidth, est)), vnorm(alpha), 620);
    this.pendingPlant = this.pendingPlant.filter((p) => p.tOff > obs.timestamp);
    return cmd;
  }

  private predictedTorque(pulse: number[], est: Estimate): Vec3 {
    const cols = torqueColumns(this.plant, est.s, est.th1, est.th2, est.fuel, this.estimator.etaT);
    let tau: Vec3 = [0, 0, 0];
    for (let i = 0; i < 6; i++) {
      const w = pulse[i] ?? 0;
      if (w <= 0) continue;
      tau = [tau[0] + cols[i]![0] * (w / CTRL_DT), tau[1] + cols[i]![1] * (w / CTRL_DT), tau[2] + cols[i]![2] * (w / CTRL_DT)];
    }
    return tau;
  }

  private replan(t: number, est: Estimate): PulsePrimitive {
    this.replanCount += 1;
    this.lastReplanT = t;
    const isolated = [...this.fdir.isolated];
    const ms = massState(this.plant, est.s, est.th1, est.th2, est.fuel);
    const state: RolloutState = rolloutFromSimLike({
      time: t,
      q: est.q,
      w: est.w,
      s: est.s,
      sd: est.sd,
      th1: est.th1,
      th1d: est.th1d,
      th2: est.th2,
      th2d: est.th2d,
      fuel: est.fuel,
      pendingPulses: this.pendingPlant,
    });
    const params: RolloutParameters = {
      inertiaEstimate: ms.Icm,
      etaTEstimate: est.etaT,
      c1Estimate: est.c1,
      c2Estimate: est.c2,
      k12Estimate: est.k12,
      failedThrusterBeliefs: isolated,
    };
    const beamCfg: BeamPlannerConfig = {
      ...DEFAULT_BEAM_CONFIG,
      horizonS: this.flightCfg.planningHorizonS,
      beamWidth: this.flightCfg.beamWidth,
      fuelFloorKg: this.flightCfg.fuelFloorKg,
    };

    const attDeg = deg(attitudeErrorAngle(qnormalize(est.q), this.plant.qTarget));
    const wmag = vnorm(est.w);
    if (est.fuel < this.flightCfg.fuelFloorKg + beamCfg.fuelReserveKg) {
      this.fallbackCount += 1;
      const coast = this.coastPrimitive();
      if (wmag > DETUMBLE_SAFE) {
        const det = this.minDetumble(state, params);
        this.lastDiag = {
          expandedNodes: 0,
          retainedNodes: 0,
          nodesPrunedForFuel: 0,
          minimumPredictedFuel: est.fuel,
          reserveKg: beamCfg.fuelReserveKg,
          selectedPlanFuelMargin: est.fuel - this.flightCfg.fuelFloorKg,
          selectedPrimitiveId: det.id,
          predictedTerminalAttitudeErrorDeg: attDeg,
          predictedTerminalAngularSpeedRadS: wmag,
          predictedTerminalFuelKg: est.fuel,
          fallback: true,
          reason: "fuel-floor-detumble-or-coast",
        };
        return det.thrusterIds.length === 0 || est.fuel < this.flightCfg.fuelFloorKg ? coast : det;
      }
      this.lastDiag = {
        expandedNodes: 0,
        retainedNodes: 0,
        nodesPrunedForFuel: 0,
        minimumPredictedFuel: est.fuel,
        reserveKg: beamCfg.fuelReserveKg,
        selectedPlanFuelMargin: est.fuel - this.flightCfg.fuelFloorKg,
        selectedPrimitiveId: coast.id,
        predictedTerminalAttitudeErrorDeg: attDeg,
        predictedTerminalAngularSpeedRadS: wmag,
        predictedTerminalFuelKg: est.fuel,
        fallback: true,
        reason: "fuel-floor-coast",
      };
      return coast;
    }

    try {
      const result = planBeam(state, params, this.plant, beamCfg);
      this.lastDiag = result.diagnostics;
      this.expandedNodes += result.diagnostics.expandedNodes;
      if (result.diagnostics.fallback) this.fallbackCount += 1;
      if (result.primitive.thrusterIds.some((id) => this.fdir.isolated.has(id))) {
        this.fallbackCount += 1;
        return this.coastPrimitive();
      }
      return result.primitive;
    } catch {
      this.fallbackCount += 1;
      if (wmag > DETUMBLE_SAFE) return this.minDetumble(state, params);
      return this.coastPrimitive();
    }
  }

  private coastPrimitive(): PulsePrimitive {
    return generatePulsePrimitives(THRUSTERS, {
      isolatedThrusters: this.fdir.isolated,
      durationsS: [MIN_PULSE as PulseDurationS],
    }).find((p) => p.thrusterIds.length === 0)!;
  }

  private minDetumble(state: RolloutState, params: RolloutParameters): PulsePrimitive {
    const isolated = this.fdir.isolated;
    const all = generatePulsePrimitives(THRUSTERS, {
      isolatedThrusters: isolated,
      durationsS: [MIN_PULSE as PulseDurationS],
    });
    const w = state.omegaB;
    let best: PulsePrimitive | null = null;
    let bestDot = 0;
    const ms = massState(this.plant, state.sliderS, state.theta1, state.theta2, state.fuelMass);
    for (const p of all) {
      if (p.thrusterIds.length !== 1) continue;
      const tau = netWrenchForPrimitive(p, THRUSTERS, params.etaTEstimate, ms.rCmB, this.plant.maxThrust).torqueB;
      const proj = -(tau[0] * w[0] + tau[1] * w[1] + tau[2] * w[2]);
      if (proj > bestDot) {
        bestDot = proj;
        best = p;
      }
    }
    return best ?? this.coastPrimitive();
  }

  diagnostics(): Readonly<ControllerDiagnostics> {
    const est = this.estimator.snapshot();
    const d = this.lastDiag;
    return {
      mode: "discrete-pulse-v2",
      observationTime: this.lastObs?.timestamp ?? 0,
      estimatedFuelKg: est.fuel,
      isolatedThrusters: [...this.fdir.isolated].sort((a, b) => a - b),
      candidateCount: d?.retainedNodes ?? 0,
      expandedNodeCount: d?.expandedNodes ?? this.expandedNodes,
      selectedPrimitiveId: d?.selectedPrimitiveId ?? this.lastPlan?.id ?? null,
      predictedTerminalAttitudeErrorDeg: d?.predictedTerminalAttitudeErrorDeg ?? null,
      predictedTerminalAngularSpeedRadS: d?.predictedTerminalAngularSpeedRadS ?? null,
      predictedTerminalFuelKg: d?.predictedTerminalFuelKg ?? null,
    };
  }

  getEstimate(): Estimate {
    return this.estimator.snapshot();
  }

  getFdir(): FdirSnapshot {
    return this.fdir.snapshot();
  }

  asPlant(): AnyController {
    const self = this;
    return {
      name: this.name,
      step: (obs) => self.stepPlant(obs),
      getEstimate: () => self.getEstimate(),
      getFdir: () => self.getFdir(),
      get faultConfidence() {
        return self.fdir.faultConfidence;
      },
      get detectedFailedThruster() {
        return self.fdir.detectedFailedThruster;
      },
      get detectionTime() {
        return self.fdir.detectionTime;
      },
      get isolationTime() {
        return self.fdir.isolationTime;
      },
      get isolationConfidence() {
        return self.fdir.isolationConfidence;
      },
    };
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
}

void commandFromPrimitive;
void CTRL_DT;
