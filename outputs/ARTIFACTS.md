# Generated artifacts

Keep aggregate JSON and this manifest in git. Full closed-loop traces
are local or release artifacts, not review surface.

Physics baseline: `bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4`

Research phase: **COMPLETE**. See `docs/research-phase.md` and `docs/benchmark.md`.
Do not treat study JSON as a flight controller.

| Artifact | Generate |
|---|---|
| `outputs/conservation.json` | `npm run test:physics` |
| `outputs/rollout-error-envelope.json` | `npm run eval:rollout-error` |
| `outputs/eval-v2-train10.json` | `npx tsx src/sim/cli/run-eval.ts --controller discrete-pulse-v2 --set train --count 10 --out outputs/eval-v2-train10.json` |
| `outputs/v2-failure-traces/` | `npm run eval:traces` |
| `outputs/capture-reachability-study.json` | `npm run eval:capture-reachability` |
| `outputs/reachability-dataset.json` | `npm run eval:reachability-dataset` |
| `src/sim/control/data/capture-value-knn.v1.json` | authoritative k-NN table (single file) |
| `outputs/capture-value-validation.json` | grouped-split validation report |
| `outputs/capture-reachability-study.quick.json` | `npx tsx src/sim/cli/run-capture-reachability.ts --quick` |
| `outputs/belief-truth-audit-train10.json` | `npm run eval:belief-audit` |
| `outputs/action-macro-library.json` | `npm run eval:action-macros` |
| `outputs/wrench-nullspace-study.json` | `npm run eval:wrench-nullspace` |
| `outputs/robust-terminal-study.json` | `npm run eval:robust-terminal` |

Do not commit `outputs/runs/`, per-seed CSV, or multi-megabyte JSON traces.
