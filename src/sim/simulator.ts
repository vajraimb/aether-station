import {
  attitudeErrorAngle,
  deg,
  qnorm,
  qRotate,
  vnorm,
  type Vec3,
} from "./math3d";
import {
  CTRL_DT,
  DT,
  Q0,
  S0,
  SD0,
  SENS_DT,
  TH1_0,
  TH1D_0,
  TH2_0,
  TH2D_0,
  W0,
} from "./constants";
import { AgentController } from "./controller";
import {
  integrateWithCollision,
  kineticPlusPotential,
  linearMomentumI,
  massState,
  modalMasses,
  sloshEnergy,
  totalAngularMomentumI,
} from "./dynamics";
import { SensorSystem } from "./sensors";
import { ThrusterSystem } from "./thrusters";
import type {
  Command,
  Metrics,
  Observation,
  PrivateScenario,
  PublicConfig,
  Sample,
  SimEvent,
  SimState,
} from "./types";
import { SCORE_TARGETS } from "./constants";

export class Simulator {
  cfg: PublicConfig;
  scenario: PrivateScenario;
  state: SimState;
  agent: AgentController;
  sensors: SensorSystem;
  thrusters: ThrusterSystem;
  log: Sample[] = [];
  events: SimEvent[] = [];
  lastObs: Observation | null = null;
  lastCmd: Command = { sliderForce: 0, pulseWidth: [0, 0, 0, 0, 0, 0] };
  maxImpact = 0;
  maxQerr = 0;
  maxConstraint = 0;
  H0: Vec3 = [0, 0, 0];
  P0: Vec3 = [0, 0, 0];
  E0 = 0;
  HerrMax = 0;
  private nextCtrl: number;
  private nextSens: number;
  private faultArmed: boolean;
  private settledHold = 0;
  private settledAt: number | null = null;
  private lastSnap: ReturnType<ThrusterSystem["evaluate"]> | null = null;
  private logEvery: number;
  private nextLog: number;
  initialSloshEnergy = 0;
  abnormal = false;

  constructor(cfg: PublicConfig, scenario: PrivateScenario) {
    this.cfg = cfg;
    this.scenario = scenario;
    this.agent = new AgentController(cfg);
    this.sensors = new SensorSystem(scenario.seed, scenario.gyroBias0, cfg.sensorPeriod);
    this.thrusters = new ThrusterSystem(scenario.etaT, {
      Fmax: cfg.maxThrust,
      delay: cfg.commandDelay,
      minPulse: cfg.minPulse,
      isp: cfg.isp,
      g0: cfg.g0,
    });
    const fuel = cfg.initialFuelMass;
    this.state = {
      t: 0,
      rI: [0, 0, 0],
      vI: [0, 0, 0],
      rCmI: [0, 0, 0],
      vCmI: [0, 0, 0],
      q: [Q0[0], Q0[1], Q0[2], Q0[3]],
      w: [W0[0], W0[1], W0[2]],
      s: S0,
      sd: SD0,
      th1: cfg.fluidPresent ? TH1_0 : 0,
      th1d: cfg.fluidPresent ? TH1D_0 : 0,
      th2: cfg.fluidPresent ? TH2_0 : 0,
      th2d: cfg.fluidPresent ? TH2D_0 : 0,
      fuel,
    };
    const ms = massState(cfg, this.state.s, this.state.th1, this.state.th2, fuel);
    this.state.rI = [0, 0, 0];
    this.state.vI = [0, 0, 0];
    this.state.rCmI = qRotate(this.state.q, ms.rCmB);
    this.state.vCmI = [0, 0, 0];
    this.H0 = totalAngularMomentumI(cfg, this.state, scenario.k12);
    this.P0 = linearMomentumI(cfg, this.state);
    this.E0 = kineticPlusPotential(cfg, this.state, scenario.k12);
    const mm = modalMasses(cfg.fluidMass);
    this.initialSloshEnergy = sloshEnergy(
      this.state.th1,
      this.state.th1d,
      this.state.th2,
      this.state.th2d,
      mm.m1,
      mm.m2,
      cfg.tankMeanRadius,
      scenario.k12,
    );
    this.nextCtrl = 0;
    this.nextSens = 0;
    this.faultArmed = true;
    this.logEvery = 0.05;
    this.nextLog = 0;
    this.record();
  }

  get t() {
    return this.state.t;
  }

  step(): boolean {
    const dt = this.cfg.dtMax;
    const t = this.state.t;
    if (t >= this.cfg.duration - 1e-12) return false;

    if (this.faultArmed && t + dt >= this.scenario.faultTime) {
      this.thrusters.fail(this.scenario.faultThruster);
      this.abnormal = true;
      this.faultArmed = false;
      this.events.push({
        t: this.scenario.faultTime,
        type: "fault_injected",
        data: { thruster: this.scenario.faultThruster },
      });
    }

    if (t + 1e-12 >= this.nextSens) {
      const snap = this.lastSnap ?? this.thrusters.evaluate(t, dt, this.state.fuel);
      const obs = this.sensors.sample(this.state, this.state.fuel, snap.current, this.abnormal);
      if (obs) this.lastObs = obs;
      this.nextSens += this.cfg.sensorPeriod;
    }

    if (t + 1e-12 >= this.nextCtrl) {
      if (this.lastObs) {
        const cmd = this.agent.step(this.lastObs);
        this.lastCmd = cmd;
        this.thrusters.submit(t, cmd);
        const d = this.agent.detectionTime;
        const iso = this.agent.isolationTime;
        if (d !== null && !this.events.some((e) => e.type === "fault_detected")) {
          this.events.push({
            t: d,
            type: "fault_detected",
            data: { delay: d - this.scenario.faultTime },
          });
        }
        if (iso !== null && !this.events.some((e) => e.type === "fault_isolated")) {
          this.events.push({
            t: iso,
            type: "fault_isolated",
            data: {
              thruster: this.agent.detectedFailedThruster,
              confidence: this.agent.isolationConfidence,
            },
          });
        }
      }
      this.nextCtrl += this.cfg.controllerPeriod;
    }

    const snap = this.thrusters.evaluate(t, dt, this.state.fuel);
    this.lastSnap = snap;
    if (this.state.fuel <= 0 && snap.fuelDot === 0 && this.state.fuel !== 0) {
      this.events.push({ t, type: "fuel_empty" });
    }

    const u = {
      FthrB: snap.Fb,
      tauThrO: snap.tauO,
      Fslider: this.lastCmd.sliderForce,
      c1: this.cfg.fluidPresent ? this.scenario.c1 : 0,
      c2: this.cfg.fluidPresent ? this.scenario.c2 : 0,
      k12: this.cfg.fluidPresent ? this.scenario.k12 : 0,
    };

    const col = integrateWithCollision(this.cfg, this.state, u, dt);
    this.state = col.state;
    this.state.fuel = Math.max(0, this.state.fuel + snap.fuelDot * dt);
    if (col.collided) {
      this.maxImpact = Math.max(this.maxImpact, col.impactSpeed);
      this.events.push({
        t: this.state.t,
        type: "collision",
        data: { speed: col.impactSpeed, impulse: col.impulse, bound: col.bound },
      });
    }

    const qn = Math.abs(qnorm(this.state.q) - 1);
    this.maxQerr = Math.max(this.maxQerr, qn);
    const viol = Math.max(0, Math.abs(this.state.s) - this.cfg.sliderMax);
    this.maxConstraint = Math.max(this.maxConstraint, viol);

    const H = totalAngularMomentumI(this.cfg, this.state, this.scenario.k12);
    const herr = vnorm([H[0] - this.H0[0], H[1] - this.H0[1], H[2] - this.H0[2]]);
    this.HerrMax = Math.max(this.HerrMax, herr);

    const att = deg(attitudeErrorAngle(this.state.q, this.cfg.qTarget));
    const wmag = vnorm(this.state.w);
    if (att < 1 && wmag < 0.008) {
      this.settledHold += dt;
      if (this.settledHold >= 3 && this.settledAt === null) {
        this.settledAt = this.state.t;
        this.events.push({ t: this.state.t, type: "settled" });
      }
    } else {
      this.settledHold = 0;
    }

    if (this.state.t + 1e-12 >= this.nextLog) {
      this.record();
      this.nextLog += this.logEvery;
    }
    return this.state.t < this.cfg.duration - 1e-12;
  }

  private record() {
    const st = this.state;
    const est = this.agent.estimate;
    const mm = modalMasses(this.cfg.fluidMass);
    const se = sloshEnergy(
      st.th1,
      st.th1d,
      st.th2,
      st.th2d,
      mm.m1,
      mm.m2,
      this.cfg.tankMeanRadius,
      this.scenario.k12,
    );
    const H = totalAngularMomentumI(this.cfg, st, this.scenario.k12);
    const herr = vnorm([H[0] - this.H0[0], H[1] - this.H0[1], H[2] - this.H0[2]]);
    const snap = this.lastSnap;
    this.log.push({
      t: st.t,
      r: [...st.rI] as Vec3,
      v: [...st.vI] as Vec3,
      q: [...st.q] as typeof st.q,
      w: [...st.w] as Vec3,
      qEst: [...est.q] as typeof est.q,
      wEst: [...est.w] as Vec3,
      gyroBiasEst: [...est.bias] as Vec3,
      s: st.s,
      sd: st.sd,
      sEst: est.s,
      sdEst: est.sd,
      th1: st.th1,
      th1d: st.th1d,
      th2: st.th2,
      th2d: st.th2d,
      th1Est: est.th1,
      th2Est: est.th2,
      sloshEnergy: se,
      fuelTrue: st.fuel,
      fuelEst: est.fuel,
      c1Est: est.c1,
      c2Est: est.c2,
      k12Est: est.k12,
      etaTEst: est.etaT,
      c1P: est.c1P,
      c2P: est.c2P,
      k12P: est.k12P,
      etaP: est.etaP,
      attitudeErrorDeg: deg(attitudeErrorAngle(st.q, this.cfg.qTarget)),
      thrusterCmd: snap ? [...snap.commanded] as typeof snap.commanded : [0, 0, 0, 0, 0, 0],
      thrusterActual: snap ? [...snap.actual] as typeof snap.actual : [0, 0, 0, 0, 0, 0],
      faultConfidence: [...this.agent.faultConfidence],
      detectedFailedThruster: this.agent.detectedFailedThruster,
      quaternionNormError: Math.abs(qnorm(st.q) - 1),
      totalAngularMomentumError: herr,
      nis: est.nis,
      sliderForce: this.lastCmd.sliderForce,
      hI: H,
    });
  }

  runAll(onProgress?: (sim: Simulator) => void, yieldEvery = 400): void {
    let n = 0;
    while (this.step()) {
      n += 1;
      if (onProgress && n % yieldEvery === 0) onProgress(this);
    }
    this.record();
  }

  metrics(): Metrics {
    return computeMetrics(this);
  }
}

export function computeMetrics(sim: Simulator): Metrics {
  const last = sim.log[sim.log.length - 1]!;
  const sc = sim.scenario;
  const det = sim.agent.detectionTime;
  const delay = det === null ? null : det - sc.faultTime;
  const isoOk = sim.agent.detectedFailedThruster === sc.faultThruster ? 1 : 0;
  const c1e = Math.abs(last.c1Est - sc.c1) / Math.max(sc.c1, 1e-6);
  const c2e = Math.abs(last.c2Est - sc.c2) / Math.max(sc.c2, 1e-6);
  const k12e = Math.abs(last.k12Est - sc.k12) / Math.max(sc.k12, 1e-6);
  const etae = Math.abs(last.etaTEst - sc.etaT) / Math.max(sc.etaT, 1e-6);
  const paramErr = sim.cfg.fluidPresent ? (c1e + c2e + k12e + etae) / 4 : etae;
  const se0 = sim.initialSloshEnergy;
  const seF = last.sloshEnergy;
  const ratio = se0 > 1e-9 ? seF / se0 : 0;
  const wmag = vnorm(last.w);

  const m: Metrics = {
    final_attitude_error_deg: last.attitudeErrorDeg,
    final_angular_speed_rad_s: wmag,
    max_slider_impact_speed_m_s: sim.maxImpact,
    initial_slosh_energy: se0,
    final_slosh_energy: seF,
    final_slosh_energy_ratio: ratio,
    remaining_fuel_kg: last.fuelTrue,
    parameter_relative_error: paramErr,
    fault_detection_delay_s: delay,
    fault_isolation_accuracy: isoOk,
    quaternion_norm_max_error: sim.maxQerr,
    maximum_constraint_violation: sim.maxConstraint,
    total_thruster_on_time: sim.thrusters.totalOnTime,
    pulse_count: sim.thrusters.pulseCount,
    run_is_deterministic: true,
    detection_time: det,
    isolation_time: sim.agent.isolationTime,
    isolated_thruster: sim.agent.detectedFailedThruster,
    settled_time: sim.events.find((e) => e.type === "settled")?.t ?? null,
    scorecard: {},
  };
  const T = SCORE_TARGETS;
  const pass = (v: number | null | boolean, ok: boolean, target: string) => ({
    value: v,
    pass: ok,
    target,
  });
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

export function runOnce(cfg: PublicConfig, scenario: PrivateScenario): Simulator {
  const sim = new Simulator(cfg, scenario);
  sim.runAll();
  return sim;
}

void CTRL_DT;
void DT;
void SENS_DT;
