# AgentArena (internal)

Generic observation/action/scoring protocol. No AETHER physics.

Domain adapters live outside this package (`src/sim/adapters/station.ts`).
Do not import `dynamics`, `math3d`, `Simulator`, quaternions, thrusters, or slosh here.
