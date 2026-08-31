# AETHER

Failed space-station attitude recovery — a 6-DOF GNC lab.

A tumbling cylindrical station, an internal sliding mass, and a two-mode nonlinear annular-tank slosh model. Six cold-gas RCS jets (max two at once, 18 N, 40 ms min pulse, 120 ms command delay). One jet fails in flight. The flight controller sees only a noisy, delayed `Observation` — never truth, never the hidden slosh coefficients `c1 / c2 / k12 / ηT`, never a wall-clock of 73.4 s.

Demo seed `20260831`: `c1=0.137`, `c2=0.091`, `k12=0.318`, `ηT=0.873`, fault at 73.4 s on +Y.

## Install

```bash
npm install
```

## Headless (no browser / WebGL / DOM)

```bash
npm run sim -- --seed 20260831 --scenario demo
npm run score -- outputs/trajectory.csv outputs/events.jsonl
npm run test:physics
npm run benchmark -- --count 20
```

Outputs land in `outputs/`:

| File | Source |
| --- | --- |
| `trajectory.csv` | plant + estimate log |
| `events.jsonl` | collisions, FDIR, scenario (hidden params for the file scorer) |
| `metrics.json` | live scorecard + FDIR times |
| `recomputed-metrics.json` | `npm run score` — files only, no Simulator |
| `counterfactual-metrics.json` | dry-tank rerun |
| `conservation.json` | open-loop energy / H / \|q\| |
| `convergence.json` | dt = 5 / 2.5 / 1.25 ms |
| `oracle-metrics.json` | truth-state controller, same actuator limits |
| `reachability.json` | pre/post-fault allocation rank, SVD, min impulse |

## Browser demo

```bash
npm run dev
```

Begin recovery. Playback, skip-to-end, orbit the station, scorecard under Report.

```bash
npm run typecheck
npm run test:sim
```

## FDIR times

`isolationDelay = isolationTime − faultInjectionTime`. If the plant injects at 73.4 s and isolation is at 74.4 s, the delay is 1.0 s, not 0.001 s.

## Layout

```
src/sim/          dynamics, RCS, sensors, MEKF, observation-only controller, oracle, FDIR, tests
src/sim/cli/      headless sim / score / physics / benchmark
src/viz/          Three.js / R3F station scene
src/components/MissionApp.tsx
```
