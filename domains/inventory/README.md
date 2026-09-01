# Inventory domain

Partial-observation reorder lab used to test AgentArena — not a solved inventory optimizer.

## Smoke evidence

| Item | Value |
|---|---|
| Command | `npm run inventory:smoke` |
| Same via unified CLI | `npm run arena -- --domain inventory --agent reorder-point --scenario smoke` |
| Tracked aggregate | [`outputs/inventory/smoke.json`](../../outputs/inventory/smoke.json) |
| Unified run bundle | `outputs/runs/inventory-reorder-smoke/manifest.json` (+ `metrics.json`, `claims.json`) |
| Version controlled | those JSON files **yes**. `outputs/inventory/<scenario>/trajectory.csv` and `events.jsonl` **no** (gitignored) |
| Re-score | after a local smoke: `InventoryScorer.score(pathToCsv, pathToEvents)` |
| Ledger `run_id` | `inventory-reorder-smoke` |

There is no `outputs/inventory-smoke.json`.

Gates: fill rate ≥ 0.95, cash > 0, stockout days < 5, zero constraint violations, supplier-alert delay < 3. Baseline **FAIL**.
