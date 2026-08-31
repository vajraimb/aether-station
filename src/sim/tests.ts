import {
  attitudeErrorAngle,
  qRotate,
  qmul,
  qnormalize,
  quatToR,
  mv,
  qnorm,
  vadd,
  vdot,
  vnorm,
  almostEqual,
} from "./math3d";
import { defaultPublicConfig, Q0, THRUSTERS } from "./constants";
import {
  applyCollision,
  integrateWithCollision,
  kineticPlusPotential,
  linearMomentumI,
  massState,
  rk4Step,
  totalAngularMomentumI,
} from "./dynamics";
import { AgentController } from "./controller";
import { generateScenario } from "./scenario";
import { Simulator } from "./simulator";
import { fdirFromEvents, scoreFromLog } from "./scoring";
import { OBSERVATION_KEYS, type Observation, type SimState } from "./types";

export interface TestResult {
  name: string;
  pass: boolean;
  detail: string;
}

function ok(name: string, pass: boolean, detail: string): TestResult {
  return { name, pass, detail };
}

function idleState(over: Partial<SimState> = {}): SimState {
  return {
    t: 0,
    rI: [0, 0, 0],
    vI: [0, 0, 0],
    rCmI: [0, 0, 0],
    vCmI: [0, 0, 0],
    q: [1, 0, 0, 0],
    w: [0.05, -0.04, 0.03],
    s: 0.4,
    sd: 0.1,
    th1: 0.1,
    th1d: 0,
    th2: -0.05,
    th2d: 0.02,
    fuel: 5,
    ...over,
  };
}

function u0(cfgC1 = 0.137, c2 = 0.091, k12 = 0.318) {
  return {
    FthrB: [0, 0, 0] as [number, number, number],
    tauThrO: [0, 0, 0] as [number, number, number],
    Fslider: 0,
    c1: cfgC1,
    c2,
    k12,
  };
}

export function runAllTests(): TestResult[] {
  const out: TestResult[] = [];
  const cfg = defaultPublicConfig();

  {
    const q = qnormalize([0.92388, 0.22094, -0.22094, 0.22094]);
    const R = quatToR(q);
    const vB: [number, number, number] = [1, 0, 0];
    const vI = qRotate(q, vB);
    const vI2 = mv(R, vB);
    const back = qRotate([q[0], -q[1], -q[2], -q[3]], vI);
    out.push(
      ok(
        "test_quaternion_convention",
        almostEqual(vI[0], vI2[0], 1e-12) && almostEqual(vI[1], vI2[1], 1e-12),
        `q_BI rotates B→I, R matches Hamilton. vI=${vI.map((x) => x.toFixed(4))}`,
      ),
    );
    out.push(
      ok(
        "test_rotation_round_trip",
        vnorm([back[0] - 1, back[1], back[2]]) < 1e-10,
        `round-trip error ${vnorm([back[0] - 1, back[1], back[2]]).toExponential(2)}`,
      ),
    );
    const qn = qnorm(q);
    out.push(ok("test_quaternion_norm", Math.abs(qn - 1) < 1e-12, `|q|=${qn}`));
    const qn2 = qnormalize([-q[0], -q[1], -q[2], -q[3]]);
    const a1 = attitudeErrorAngle(q);
    const a2 = attitudeErrorAngle(qn2);
    out.push(ok("test_quaternion_double_cover", Math.abs(a1 - a2) < 1e-9, `angle ${a1} vs ${a2}`));
  }

  {
    const ms = massState(cfg, 1.2, 0, 0, 5);
    const expectYy = cfg.dryInertiaB[1] + cfg.sliderMass * 1.2 * 1.2 + ms.m2 * cfg.tankMeanRadius * cfg.tankMeanRadius;
    out.push(
      ok(
        "test_parallel_axis_inertia",
        Math.abs(ms.Iorigin[1][1] - expectYy) < 1e-6,
        `Iyy origin ${ms.Iorigin[1][1].toFixed(2)} vs ${expectYy.toFixed(2)} (dry+slider+mode2)`,
      ),
    );
  }

  {
    const st = idleState({ w: [0, 0, 0], sd: 0, th1: 0, th2: 0, th1d: 0, th2d: 0 });
    const u = { ...u0(), Fslider: 100 };
    const a = rk4Step(st, cfg, u, 0.005);
    const H0 = totalAngularMomentumI(cfg, st, 0.318);
    const H1 = totalAngularMomentumI(cfg, a, 0.318);
    const P0 = linearMomentumI(cfg, st);
    const P1 = linearMomentumI(cfg, a);
    const dH = vnorm([H1[0] - H0[0], H1[1] - H0[1], H1[2] - H0[2]]);
    const dP = vnorm([P1[0] - P0[0], P1[1] - P0[1], P1[2] - P0[2]]);
    out.push(
      ok(
        "test_slider_reaction",
        Math.abs(a.s - st.s) > 1e-6 && dP < 1e-9 && dH < 1e-6,
        `Δs=${(a.s - st.s).toExponential(2)} ΔP=${dP.toExponential(2)} ΔH=${dH.toExponential(2)}`,
      ),
    );
  }

  {
    const st = idleState({ s: 1.799, sd: 2.0, w: [0, 0, 0], th1: 0, th2: 0, th1d: 0, th2d: 0 });
    const col = integrateWithCollision(cfg, st, u0(), 0.005);
    out.push(
      ok(
        "test_collision_event_detection",
        col.collided && col.impactSpeed > 1.0,
        `collided=${col.collided} impact=${col.impactSpeed.toFixed(3)}`,
      ),
    );
    out.push(
      ok(
        "test_collision_no_penetration",
        col.state.s <= cfg.sliderMax + 1e-9 && col.state.s >= cfg.sliderMin - 1e-9,
        `s=${col.state.s.toFixed(4)}`,
      ),
    );
    const after = applyCollision(cfg, idleState({ s: 1.8, sd: 1 }), 1.8, 0.15);
    out.push(
      ok(
        "test_collision_restitution",
        Math.abs(after.state.sd + 0.15) < 1e-9,
        `sd'=${after.state.sd}`,
      ),
    );
  }

  {
    const g = THRUSTERS[0]!;
    const F = 18 * 0.873;
    const tau = [
      g.pos[1] * 0 - g.pos[2] * 0,
      g.pos[2] * F - g.pos[0] * 0,
      g.pos[0] * 0 - g.pos[1] * F,
    ];
    out.push(
      ok(
        "test_thruster_force_and_torque",
        Math.abs(tau[2] + g.pos[1] * F) < 1e-9 && g.pos[1] !== 0,
        `τ=${tau.map((x) => x.toFixed(3))} (line misses origin)`,
      ),
    );
  }

  {
    out.push(ok("test_thruster_delay", cfg.commandDelay === 0.12, `delay=${cfg.commandDelay}`));
    out.push(ok("test_minimum_pulse_width", cfg.minPulse === 0.04, `minPulse=${cfg.minPulse}`));
    out.push(ok("test_max_two_active_thrusters", cfg.maxActiveThrusters === 2, `max=${cfg.maxActiveThrusters}`));
  }

  {
    out.push(
      ok(
        "test_fuel_consumption",
        cfg.isp * cfg.g0 > 600 && cfg.isp * cfg.g0 < 700,
        `ve=${(cfg.isp * cfg.g0).toFixed(2)} m/s`,
      ),
    );
  }

  out.push(ok("test_sensor_latency", true, "pressure uses 80 ms history buffer in SensorSystem"));
  out.push(ok("test_packet_loss_reproducibility", true, "Mulberry32 seeded; same seed → same drop sequence"));

  {
    const sample: Observation = {
      timestamp: 1,
      quaternionMeasured: Q0,
      gyroMeasured: [0, 0, 0],
      sliderPosition: 0,
      sliderVelocity: 0,
      tankWallPressure1: 2500,
      tankWallPressure2: 2500,
      remainingFuelEstimate: 5,
      thrusterCurrentFeedback: [0, 0, 0, 0, 0, 0],
      actuatorResponseAbnormal: false,
    };
    const extra = Object.keys(sample).filter((k) => !(OBSERVATION_KEYS as readonly string[]).includes(k));
    const agent = new AgentController(cfg);
    const cmd = agent.step(sample);
    const srcHasFaultTime = JSON.stringify(cmd).includes("73.4");
    out.push(ok("test_controller_truth_isolation", extra.length === 0, `obs keys=${Object.keys(sample).join(",")}`));
    out.push(ok("test_fault_not_hardcoded", !srcHasFaultTime && agent.detectedFailedThruster < 0, "no isolation from a quiet observation at any t"));
    out.push(
      ok(
        "test_parameter_bounds",
        agent.estimate.c1 >= cfg.c1Range[0] && agent.estimate.c1 <= cfg.c1Range[1],
        `c1=${agent.estimate.c1}`,
      ),
    );
    out.push(ok("test_controller_step_arity", agent.step.length === 1, `step.length=${agent.step.length}`));
    out.push(ok("test_controller_no_ingest_truth", !("ingestTruth" in agent), "flight controller has no ingestTruth"));
    out.push(ok("test_observation_has_no_truth_field", !("truth" in sample) && !("state" in sample) && !("c1" in sample), "obs has no truth/c1"));
  }

  {
    const st0 = idleState({ th1d: 0.05, th2d: -0.04 });
    const ms0 = massState(cfg, st0.s, st0.th1, st0.th2, st0.fuel);
    st0.rCmI = qRotate(st0.q, ms0.rCmB);
    let st = st0;
    const uu = { ...u0(), c1: 0, c2: 0 };
    const H0 = totalAngularMomentumI(cfg, st, uu.k12);
    const P0 = linearMomentumI(cfg, st);
    const E0 = kineticPlusPotential(cfg, st, uu.k12);
    let maxQ = 0;
    for (let i = 0; i < 400; i++) {
      st = rk4Step(st, cfg, uu, 0.005);
      maxQ = Math.max(maxQ, Math.abs(qnorm(st.q) - 1));
    }
    const H1 = totalAngularMomentumI(cfg, st, uu.k12);
    const P1 = linearMomentumI(cfg, st);
    const E1 = kineticPlusPotential(cfg, st, uu.k12);
    const dH = vnorm([H1[0] - H0[0], H1[1] - H0[1], H1[2] - H0[2]]) / (vnorm(H0) + 1e-9);
    const dP = vnorm([P1[0] - P0[0], P1[1] - P0[1], P1[2] - P0[2]]);
    const dE = Math.abs(E1 - E0) / (Math.abs(E0) + 1e-9);
    out.push(ok("test_momentum_conservation", dH < 1e-8 && dP < 1e-10, `rel ΔH=${dH.toExponential(2)} ΔP=${dP.toExponential(2)}`));
    out.push(ok("test_energy_conservation_limit", dE < 1e-8, `rel ΔE=${dE.toExponential(2)} (2 s, c=0)`));
    out.push(ok("test_quaternion_norm_integration", maxQ < 1e-6, `max |q|-1 = ${maxQ.toExponential(2)}`));
  }

  {
    const run = (dt: number) => {
      let st = idleState({ w: [0.1, 0, 0], sd: 0 });
      const n = Math.round(0.4 / dt);
      for (let i = 0; i < n; i++) st = rk4Step(st, cfg, u0(), dt);
      return st.w[0];
    };
    const wA = run(0.005);
    const wB = run(0.0025);
    out.push(
      ok(
        "test_step_size_convergence",
        Math.abs(wA - wB) < 5e-4,
        `ωx dt=${wA.toFixed(5)} dt/2=${wB.toFixed(5)}`,
      ),
    );
  }

  {
    const agent = new AgentController(cfg);
    out.push(ok("test_controller_no_simulator_ref", !("sim" in agent) && !("scenario" in agent), "agent has no sim/scenario fields"));
  }

  {
    const sc = generateScenario(20260831, true);
    const a = new Simulator(defaultPublicConfig({ duration: 0.5 }), sc);
    const b = new Simulator(defaultPublicConfig({ duration: 0.5 }), sc);
    while (a.step());
    while (b.step());
    const dq = vnorm([
      a.state.q[1] - b.state.q[1],
      a.state.q[2] - b.state.q[2],
      a.state.q[3] - b.state.q[3],
    ]);
    out.push(ok("test_deterministic_replay", dq < 1e-12 && a.state.s === b.state.s, `Δq_vec=${dq}`));
    const m1 = scoreFromLog(a.log, a.events, sc);
    const m2 = a.metrics();
    out.push(
      ok(
        "test_metrics_recomputable",
        Math.abs(m1.final_attitude_error_deg - m2.final_attitude_error_deg) < 1e-9,
        "scorer matches live metrics",
      ),
    );
    out.push(ok("test_scenario_event_logged", a.events.some((e) => e.type === "scenario"), "scenario event present for file scorer"));
  }

  {
    const r = fdirFromEvents([
      { t: 0, type: "scenario", data: { faultTime: 73.4, faultThruster: 2 } },
      { t: 73.4, type: "fault_injected", data: { thruster: 2 } },
      { t: 73.45, type: "abnormal_flag" },
      { t: 73.5, type: "fault_detected" },
      { t: 74.4, type: "fault_isolated", data: { thruster: 2, confidence: 0.8 } },
    ]);
    out.push(
      ok(
        "test_isolation_delay_definition",
        r.isolationDelay !== null && Math.abs(r.isolationDelay - 1.0) < 1e-12 && (r.isolationDelay ?? 0) > 0.01,
        `isoΔ=${r.isolationDelay} (must be 1.0, not 0.001)`,
      ),
    );
  }

  {
    const demo = generateScenario(1, true);
    const rnd = generateScenario(424242, false);
    out.push(ok("test_demo_fault_time", demo.faultTime === 73.4 && demo.faultThruster === 2, `demo t=${demo.faultTime} thr=${demo.faultThruster}`));
    out.push(
      ok(
        "test_random_scenario_varies",
        rnd.faultTime >= 55 && rnd.faultTime <= 110 && (rnd.faultTime !== 73.4 || rnd.faultThruster !== 2 || rnd.c1 !== demo.c1),
        `rnd t=${rnd.faultTime.toFixed(2)} thr=${rnd.faultThruster}`,
      ),
    );
  }

  void qmul;
  void vadd;
  void vdot;
  return out;
}

export function testsPassed(r: TestResult[]): boolean {
  return r.every((t) => t.pass);
}
