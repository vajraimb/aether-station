# AETHER

Failed space-station attitude recovery — a 6-DOF GNC lab.

A tumbling cylindrical station, an internal sliding mass, and a two-mode nonlinear annular-tank slosh model. Six cold-gas RCS jets (max two at once, 18 N, 40 ms min pulse, 120 ms command delay). The **+Y** thruster fails in flight. The controller sees only a noisy, delayed observation — never truth, never the hidden slosh coefficients `c1 / c2 / k12 / ηT`, never a wall-clock of 73.4 s.

Demo seed `20260831`: `c1=0.137`, `c2=0.091`, `k12=0.318`, `ηT=0.873`.

## Run

```bash
npm install
npm run dev
```

Then open the app and **Begin recovery**. Playback 1/4/8/16×, skip-to-end, orbit the station, and read the scorecard under **Report**.

```bash
npm run typecheck
npm run test:sim              # 26 physics / isolation checks
npx tsx src/sim/selftest.ts --mission   # full 180 s demo
```

## Physics

- Right-handed frames. Quaternion `[w, x, y, z]`. `q_BI` is the active rotation taking body vectors to inertial: `v_I = R(q) v_B`. `ω` is in B.
- Newton–Euler about the origin with `I(s, θ)`, `İω`, and slosh `h_rel`.
- Two equal modal-mass pendulums (0.4 `m_fluid` each) so the `k12` coupling potential is energy-consistent. Tank-wall restoring `ω_i² sin θ`, not gravity.
- RK4 `Δt = 5 ms`. Slider end-stops are event-located inelastic collisions (`e = 0.15`).
- Internal EOMs drop ~5 % CM-recoil (documented in `src/sim/dynamics.ts`).

## Agent (observation only)

MEKF attitude + gyro bias, complementary slider, delayed pressure inversion for slosh, bounded RLS-style parameters. Hierarchical PD / rate damping, two-thruster allocation. FDIR from delayed command vs current residual — never keyed off 73.4 s.

## Scorecard (demo seed, 180 s)

| Gate | Result |
| --- | --- |
| Attitude error | ~2.35° (target < 1°) |
| \|ω\| | ~5.7×10⁻⁴ rad/s |
| Slider impact | 0 |
| Slosh energy ratio | ≪ 0.08 |
| Fuel remaining | ~3.16 kg |
| Param relative error | ~0.09 |
| FDIR | isolates +Y at t ≈ 74.4 s |
| \|q\| − 1 | ~10⁻¹⁶ |

Capture reaches ~1° around t = 70–75 s, then walks. After +Y is lost, a 40 ms min-pulse on the remaining pair is Δω ≈ 4×10⁻⁴ rad/s; holding 1° for the last ~80 s would need a third jet or a shorter pulse. Details are on the in-app **Model** tab.

## Layout

```
src/sim/     6-DOF dynamics, RCS, sensors, MEKF, controller, FDIR, tests
src/viz/     Three.js / R3F station scene
src/components/MissionApp.tsx   briefing, HUD, telemetry, scorecard
```
