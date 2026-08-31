import { attitudeErrorAngle, deg } from "./math3d.ts";
import { defaultPublicConfig } from "./constants.ts";
import { generateScenario } from "./scenario.ts";
import { Simulator } from "./simulator.ts";
import { runAllTests } from "./tests.ts";

const results = runAllTests();
let fail = 0;
for (const r of results) {
  const mark = r.pass ? "PASS" : "FAIL";
  if (!r.pass) fail += 1;
  console.log(`${mark}  ${r.name}  ${r.detail}`);
}
console.log(`\n${results.length - fail}/${results.length} passed`);

if (process.argv.includes("--mission")) {
  const t0 = Date.now();
  const cfg = defaultPublicConfig();
  const sc = generateScenario(20260831, true);
  const sim = new Simulator(cfg, sc);
  const marks = [5, 10, 20, 40, 70, 80, 90, 97, 110, 140, 180];
  let mi = 0;
  while (sim.step()) {
    if (mi < marks.length && sim.state.t >= marks[mi]!) {
      const s = sim.log[sim.log.length - 1]!;
      const w = Math.hypot(s.w[0], s.w[1], s.w[2]);
      const estAtt = deg(attitudeErrorAngle(s.qEst));
      const wEst = Math.hypot(s.wEst[0], s.wEst[1], s.wEst[2]);
      console.log(
        `t=${s.t.toFixed(2)} att=${s.attitudeErrorDeg.toFixed(2)} estAtt=${estAtt.toFixed(2)} |w|=${w.toFixed(4)} |wEst|=${wEst.toFixed(4)} fuel=${s.fuelTrue.toFixed(3)} iso=${s.detectedFailedThruster}`,
      );
      mi += 1;
    }
  }
  const m = sim.metrics();
  console.log("\n--- mission 180 s ---");
  console.log(JSON.stringify(m.scorecard, null, 2));
  console.log("elapsed_ms", Date.now() - t0);
  console.log("samples", sim.log.length, "events", JSON.stringify(sim.events.map((e) => ({ t: e.t, type: e.type, data: e.data }))));
  console.log("onTime", sim.thrusters.totalOnTime, "pulses", sim.thrusters.pulseCount);
}

process.exit(fail ? 1 : 0);
