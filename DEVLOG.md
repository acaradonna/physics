# Dev Log

2025-10-24

- Added per-body radius and sphere-sphere narrowphase baseline.
- Implemented minimal projection solver to remove interpenetration.
- Created web demo to visualize sphere separation; verified determinism tests pass.
- Next: Move to a frictionless PGS solver with warm-starting for stability under gravity and stacking.
- Notes: Keep pair ordering stable; ensure warm-start mapping is deterministic.

2025-10-24 (later)

- Reworked step to introduce a frictionless PGS velocity solver with Baumgarte positional bias and warm-starting.
- Added `getVelocity` for testing and WASM/JS paths.
- New `solver_velocity` test asserts non-closing normal velocity after iterations.
- Warm-start data stored per pair (a<b) with accumulated normal impulses.
- Future: add tangential friction, contact manifolds, clamping by effective mass.

2025-10-24 (even later)

- Added friction and restitution material properties to `RigidBodyDesc` and `Contact`.
- Implemented Coulomb friction (tangential impulse) in PGS solver with friction cone clamping.
- Implemented restitution (bounce) by applying restitution bias on first iteration when closing velocity exceeds threshold.
- Combined material properties: geometric mean for friction, minimum for restitution (standard practice).
- Extended warm-start to store both normal and tangential impulses per pair.
- Added `restitution` test: bouncy ball dropped from height validates positive velocity after bounce.
- Research note: Restitution is velocity-dependent; only applied when vn < -threshold to avoid jitter at rest.
- Research note: Friction cone clamped by |Jt| <= friction * Jn ensures Coulomb law compliance.
