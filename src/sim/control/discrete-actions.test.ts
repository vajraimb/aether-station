import { FMAX, MIN_PULSE, THRUSTERS } from "../constants";
import { vcross, vnorm, type Vec3 } from "../math3d";
import {
  PULSE_DURATIONS_S,
  commandFromPrimitive,
  filterExecutablePrimitives,
  generatePulsePrimitives,
  isLegalPulsePrimitive,
  leavesFuelFloor,
  netWrenchForPrimitive,
} from "./discrete-actions";

const assert = (condition: unknown, message: string): void => {
  if (!condition) throw new Error(message);
};

export function runDiscreteActionTests(): readonly string[] {
  const passed: string[] = [];
  const all = generatePulsePrimitives(THRUSTERS);
  const expectedPerDuration = 6 + (6 * 5) / 2;
  const nCoast = PULSE_DURATIONS_S.length;
  assert(
    all.length === nCoast + PULSE_DURATIONS_S.length * expectedPerDuration,
    `unexpected primitive count ${all.length}`,
  );
  passed.push("test_primitive_count");

  assert(new Set(all.map((primitive) => primitive.id)).size === all.length, "primitive ids are not unique");
  passed.push("test_primitive_unique_ids");

  assert(all.every((primitive) => isLegalPulsePrimitive(primitive, THRUSTERS)), "illegal primitive generated");
  passed.push("test_primitive_legality");

  const isolated = new Set([2, 5]);
  const degraded = generatePulsePrimitives(THRUSTERS, { isolatedThrusters: isolated });
  assert(
    degraded.every((primitive) => primitive.thrusterIds.every((id) => !isolated.has(id))),
    "isolated thruster remained in action set",
  );
  passed.push("test_failed_thruster_excluded");

  const pairs = degraded.filter((primitive) => primitive.thrusterIds.length === 2);
  assert(pairs.every((primitive) => primitive.thrusterIds[0]! < primitive.thrusterIds[1]!), "pairs are not canonical");
  passed.push("two-thruster pairs are canonical");

  const shortestSingle = all.find(
    (primitive) => primitive.thrusterIds.length === 1 && primitive.durationS === PULSE_DURATIONS_S[0],
  );
  assert(shortestSingle, "shortest single pulse missing");
  assert(!leavesFuelFloor(shortestSingle!, 2.8, 2.8), "fuel-floor violation was accepted");
  assert(leavesFuelFloor(shortestSingle!, 3.0, 2.8), "feasible pulse was rejected");
  passed.push("fuel floor is a hard feasibility check");

  const short = all.find((p) => p.durationS + 1e-12 < MIN_PULSE);
  assert(!short, "sub-minimum pulse width generated");
  passed.push("test_minimum_pulse_width");

  const pending = [{ id: 0, tOn: 0.12, tOff: 0.44 }];
  const filtered = filterExecutablePrimitives(all, {
    nowS: 0,
    commandDelayS: 0.12,
    pendingPulses: pending,
    isolatedThrusters: new Set(),
    estimatedFuelKg: 4.5,
    fuelFloorKg: 2.8,
    reserveKg: 0.08,
    maxActive: 2,
  });
  assert(filtered.some((p) => p.thrusterIds.length === 0), "coast was dropped");
  assert(
    filtered.every((p) => p.thrusterIds.length === 0 || !p.thrusterIds.includes(0)),
    "pending thruster 0 still offered",
  );
  const pairWithPending = filtered.filter((p) => p.thrusterIds.length === 2);
  assert(pairWithPending.length === 0, "concurrency filter leaked a 2-jet while one jet is occupied");
  passed.push("test_pending_concurrency_filter");

  const dry = filterExecutablePrimitives(all, {
    nowS: 0,
    commandDelayS: 0.12,
    pendingPulses: [],
    isolatedThrusters: new Set(),
    estimatedFuelKg: 2.81,
    fuelFloorKg: 2.8,
    reserveKg: 0.08,
    maxActive: 2,
  });
  assert(dry.every((p) => p.thrusterIds.length === 0), "fuel-floor filter leaked a firing primitive");
  passed.push("test_fuel_floor_pruning");

  const one = all.find((p) => p.thrusterIds.length === 1 && p.thrusterIds[0] === 0 && p.durationS === 0.04)!;
  const rCmB: Vec3 = [0.05, 0.01, -0.02];
  const wrench = netWrenchForPrimitive(one, THRUSTERS, 0.87, rCmB, FMAX);
  const geom = THRUSTERS[0]!;
  const F: Vec3 = [0.87 * FMAX * geom.dir[0], 0.87 * FMAX * geom.dir[1], 0.87 * FMAX * geom.dir[2]];
  const r: Vec3 = [geom.pos[0] - rCmB[0], geom.pos[1] - rCmB[1], geom.pos[2] - rCmB[2]];
  const tau = vcross(r, F);
  const torqueErr = vnorm([wrench.torqueB[0] - tau[0], wrench.torqueB[1] - tau[1], wrench.torqueB[2] - tau[2]]);
  const forceErr = vnorm([wrench.forceB[0] - F[0], wrench.forceB[1] - F[1], wrench.forceB[2] - F[2]]);
  assert(torqueErr < 1e-12 && forceErr < 1e-12, `wrench mismatch τ=${torqueErr} F=${forceErr}`);
  assert(Math.abs(wrench.forceB[0]) > 1 && Math.abs(wrench.torqueB[2]) > 1, "force/torque collapsed to the same axis");
  passed.push("test_primitive_wrench");

  const cc = commandFromPrimitive(one, 12);
  assert(cc.thrusters.length === 1 && cc.thrusters[0]!.pulseWidthS === 0.04 && cc.sliderForceN === 12, "command mapping");
  passed.push("commandFromPrimitive maps pulse width");

  const ids = all.map((p) => p.id).join("\n");
  const again = generatePulsePrimitives(THRUSTERS).map((p) => p.id).join("\n");
  assert(ids === again, "primitive order is not deterministic");
  passed.push("primitive sort is deterministic");

  return passed;
}
