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
