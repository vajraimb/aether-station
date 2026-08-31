import { defaultPublicConfig } from "../constants";
import {
  ENVELOPE_HORIZONS_S,
  measureEnvelope,
  publicStateSpecs,
  stageForHorizon,
  STAGE_TOLERANCE,
  type EnvelopeReport,
} from "./rollout-error";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runRolloutErrorTests(): T[] {
  const out: T[] = [];
  const specs = publicStateSpecs();
  check(
    "test_envelope_public_state_grid",
    specs.length === 12 &&
      specs.some((s) => s.regime === "high-rate") &&
      specs.some((s) => s.fault === "one-isolated") &&
      specs.some((s) => s.pending === "non-empty"),
    `n=${specs.length}`,
    out,
  );
  check("test_stage_horizon_split", stageForHorizon(0.5) === "terminal" && stageForHorizon(5) === "guidance" && stageForHorizon(8) === "long", "0.5/5/8", out);

  const plant = defaultPublicConfig();
  const short: EnvelopeReport = measureEnvelope(plant, {
    horizons: [0.5, 2],
    actionClasses: ["coast", "single"],
  });
  check("test_envelope_cells_cover_axes", short.cells.length === specs.length * 2 * 2, `cells=${short.cells.length}`, out);
  const half = short.byHorizon.find((g) => g.key === "horizon:0.5");
  check(
    "test_envelope_short_horizon_finite",
    Boolean(half && Number.isFinite(half.attRad.p50) && half.attRad.p50 < 0.05),
    `p50=${half?.attRad.p50}`,
    out,
  );
  const two = short.byHorizon.find((g) => g.key === "horizon:2");
  check(
    "test_envelope_grows_with_horizon",
    Boolean(half && two && two.attRad.p50 + 1e-12 >= half.attRad.p50 * 0.5),
    `0.5s=${half?.attRad.p50} 2s=${two?.attRad.p50}`,
    out,
  );
  check(
    "test_terminal_tolerance_stricter_than_gate",
    STAGE_TOLERANCE.terminal.attRad < Math.PI / 180,
    `tol=${STAGE_TOLERANCE.terminal.attRad}`,
    out,
  );
  check("test_envelope_horizon_list", ENVELOPE_HORIZONS_S.length === 7 && ENVELOPE_HORIZONS_S[0] === 0.5 && ENVELOPE_HORIZONS_S[6] === 10, ENVELOPE_HORIZONS_S.join(","), out);
  return out;
}
