# Experiment ledger

Unified run records live in `src/sim/ledger/` and `outputs/ledger/`.

```bash
npm run ledger:stamp
```

writes `outputs/ledger/index.json` plus one file per `run_id`.

## Schema

`RunRecord` (`src/sim/ledger/schema.ts`): `run_id`, `kind`, `status`, `code_sha`, `physics_sha`, `parent_run_id`, `metrics`, `artifacts`, `reproduction_command`, `claims[]`.

`claim_type`:

| Type | Meaning |
|---|---|
| `measured` | Observed on a pinned SHA |
| `search_unreached` | Search did not find a feasible plan under budget |
| `proven` | Reserved. Do not use for search misses |
| `hypothesis` | Unverified |
| `deprecated` | Line closed |

A search miss is **not** physical unreachability.

Lineage is `parent_run_id`. JSON catalog is the source of truth. SQLite ingest:

```bash
npm run ledger:ingest
```

writes `outputs/ledger/experiments.sqlite` (gitignored). Same schema fields (`domain`, `environment_version`, `agent_version`, `scenario_hash`, `artifact_manifest`, `claim_type`, `failure_class`) are intended for later inventory / Bid Pitwall runs. Do not create a second database.
