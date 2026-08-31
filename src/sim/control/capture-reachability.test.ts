import { defaultPublicConfig } from "../constants";
import { attitudeErrorAngle, deg, qnormalize } from "../math3d";
import { rolloutFromSimLike } from "./rollout-model";
import {
  capturedGates,
  harvestStudyStates,
  scaleAttitude,
  searchState,
  STUDY_BUCKETS_DEG,
} from "./capture-reachability";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runCaptureReachabilityTests(): T[] {
  const out: T[] = [];
  const plant = defaultPublicConfig();

  {
    const harvested = harvestStudyStates(plant);
    check("test_harvest_20_states", harvested.length === 20, `n=${harvested.length}`, out);
    const buckets = [...new Set(harvested.map((s) => s.bucketDeg))].sort((a, b) => b - a);
    check(
      "test_harvest_buckets",
      buckets.join(",") === STUDY_BUCKETS_DEG.join(","),
      buckets.join(","),
      out,
    );
    const attOk = harvested.every((s) => Math.abs(s.bucketDeg - deg(attitudeErrorAngle(s.state.qBI, plant.qTarget))) < 0.05);
    check("test_harvest_attitudes_match_buckets", attOk, "bucket vs q error", out);
  }

  {
    const st = rolloutFromSimLike({
      time: 0,
      q: qnormalize([0.9962, 0, 0.0872, 0]),
      w: [0, 0.01, 0],
      s: 0.1,
      sd: 0,
      th1: 0.02,
      th1d: 0,
      th2: 0,
      th2d: 0,
      fuel: 3.4,
    });
    const scaled = scaleAttitude(st, 5, plant.qTarget);
    const got = deg(attitudeErrorAngle(scaled.qBI, plant.qTarget));
    check("test_scale_attitude_5deg", Math.abs(got - 5) < 0.05, `got=${got.toFixed(3)}`, out);
    check(
      "test_scale_attitude_preserves_rate_fuel",
      Math.abs(scaled.omegaB[1] - 0.01) < 1e-12 && scaled.fuelMass === 3.4,
      `w1=${scaled.omegaB[1]} fuel=${scaled.fuelMass}`,
      out,
    );
  }

  {
    const inGate = rolloutFromSimLike({
      time: 0,
      q: [1, 0, 0, 0],
      w: [0.001, 0, 0],
      s: 0,
      sd: 0,
      th1: 0,
      th1d: 0,
      th2: 0,
      th2d: 0,
      fuel: 3.2,
    });
    const g = capturedGates(inGate, plant.qTarget);
    check("test_captured_gates_conjunction_true", g.captured && g.attOk && g.rateOk && g.fuelOk, JSON.stringify(g), out);

    const attOnly = rolloutFromSimLike({
      time: 0,
      q: [1, 0, 0, 0],
      w: [0.05, 0, 0],
      s: 0,
      sd: 0,
      th1: 0,
      th1d: 0,
      th2: 0,
      th2d: 0,
      fuel: 3.2,
    });
    const a = capturedGates(attOnly, plant.qTarget);
    check("test_captured_gates_rejects_att_ball_alone", a.attOk && !a.rateOk && !a.captured, `att=${a.attDeg} w=${a.omega}`, out);

    const fuelFail = rolloutFromSimLike({
      time: 0,
      q: [1, 0, 0, 0],
      w: [0.001, 0, 0],
      s: 0,
      sd: 0,
      th1: 0,
      th1d: 0,
      th2: 0,
      th2d: 0,
      fuel: 2.79,
    });
    const f = capturedGates(fuelFail, plant.qTarget);
    check("test_captured_gates_rejects_fuel_floor", f.attOk && f.rateOk && !f.fuelOk && !f.captured, `fuel=${f.fuelKg}`, out);
  }

  {
    const harvested = harvestStudyStates(plant).find((s) => s.bucketDeg === 1.5 && s.rateMode === "closing" && s.fault === "healthy")!;
    const r = searchState(harvested, plant, "eigen", 2, 8);
    check(
      "test_search_does_not_claim_peek_capture_without_commit",
      r.captured === false || (r.captureTimeS != null && r.captureTimeS >= 0),
      `captured=${r.captured} t=${r.captureTimeS} note=${r.note}`,
      out,
    );
    check("test_search_reports_fuel_and_omega", r.finalFuelKg > 2.8 && Number.isFinite(r.finalOmega), `fuel=${r.finalFuelKg} w=${r.finalOmega}`, out);
  }

  return out;
}
