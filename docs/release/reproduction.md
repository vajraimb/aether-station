# Reproduction — AETHER Benchmark v0.1.0

Pinned references (prefer tags and hashes over branch names):

| Ref | Value |
|---|---|
| Release tag | `v0.1.0-benchmark` |
| Freeze tag | `benchmark/research-phase-complete-v1` |
| Freeze commit | `4b5d7a6b2450f3e4613e50a9d77d9a5dfbaf15e4` |
| Physics kernel | `bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4` |
| Branch (moving) | `release/v0.1.0-benchmark` |

## Commands

```bash
git checkout v0.1.0-benchmark
npm ci
npm run check
npm run test:physics -- --full
npm run eval -- --controller baseline --set smoke
npm run release:manifest
```

`npm run build` is the web demo. It is not required to score the plant.

## Expected

| Check | Result |
|---|---|
| `git diff bdfff5b -- src/sim/math3d.ts src/sim/dynamics.ts src/sim/audit.ts` | empty |
| `test:physics --full` | 270/270 PASS |
| baseline smoke | runner completes; **control FAIL is expected** |
| `outputs/release/release-manifest.json` | physics SHA + artifact checksums |

## Artifact hashes

After `npm run release:manifest`, compare `outputs/release/checksums.sha256`.

## Closed claims (do not re-run as if open)

See `docs/research-phase.md` and archive tags. Hidden set stays blocked.
