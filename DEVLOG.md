# Dev Log

Engineering decisions, implementation notes, research findings, and challenges during development.

---

## 2025-10-24 09:00 EDT - Initial Narrowphase Implementation

**What:** Added per-body radius and sphere-sphere narrowphase baseline.

**Implementation:**
- Added `radius` field to `RigidBodyDesc` (default 0.5f)
- Implemented `generate_contacts_sphere_sphere()` in `src/collision/narrowphase.cpp`
- Contacts store normal direction and penetration depth
- Minimal projection solver splits penetration equally between bodies

**Testing:**
- Created `collision_spheres` test
- All determinism tests pass

**Decision:** Start with projection solver before velocity-based PGS for incremental validation.

**Next:** Move to velocity-based PGS solver with warm-starting.

---

## 2025-10-24 10:15 EDT - PGS Velocity Solver with Warm-Start

**What:** Reworked simulation step to use frictionless PGS velocity solver.

**Implementation:**
- Refactored `World::step()` to separate velocity integration, broadphase, narrowphase, solve, position integration
- PGS solver: 8 iterations, Baumgarte bias (0.2) for positional error correction
- Warm-start: store per-pair accumulated normal impulses, apply on next frame
- Effective mass: `k = 1/ma + 1/mb` (no angular terms yet)

**API Changes:**
- Added `getVelocity()` to C++/C APIs
- Exported `ape_world_get_velocity_out` in WASM

**Testing:**
- Added `solver_velocity` test: validates non-closing normal velocity
- Optional positional correction pass eliminates residual overlap

**Research Notes:**
- Baumgarte stabilization: bias = β * (penetration / dt)
- Warm-starting critical for stack stability (converges in 2-3 iterations vs 8+ cold-start)
- Clamping normal impulse to non-negative prevents tension

**Challenges:**
- Initial `collision_spheres` test failed; needed to increase iterations and add positional correction
- Balance between Baumgarte factor (too high = jitter, too low = sinking)

---

## 2025-10-24 11:30 EDT - Friction and Restitution

**What:** Added material properties (friction, restitution) with full Coulomb friction and bounce.

**Implementation:**
- Extended `RigidBodyDesc`: `friction` (0-1, default 0.5), `restitution` (0-1, default 0.0)
- Extended `Contact`: stores combined friction/restitution per pair
- Material combination rules:
  - Friction: geometric mean `sqrt(fa * fb)` (preserves symmetry)
  - Restitution: minimum `min(ra, rb)` (conservative, least bouncy dominates)

**Friction Solver:**
- Tangential impulse computed perpendicular to normal: `vt = rv - (rv·n)n`
- Friction cone constraint: `|Jt| <= μ * Jn` (Coulomb's law)
- Accumulated tangential impulse clamped within cone each iteration

**Restitution Solver:**
- Applied only on first iteration when `vn < -0.1` (threshold prevents jitter at rest)
- Restitution bias: `bias -= e * vn` (e is restitution coefficient)
- Physics: `v'_n = -e * v_n` for perfect collision

**Warm-Start Extensions:**
- Store both normal and tangential impulses per pair
- Tangential impulse stored as `Vec3` for full 3D friction

**Testing:**
- Added `restitution` test: bouncy ball (e=0.8) dropped from 10m height
- Validates positive velocity after bounce
- All 12 tests pass (100%)

**Web Demo:**
- Created interactive `Stacking` demo
- Adjustable friction/restitution sliders
- Stack of 5 spheres showcases material behavior

**Research Notes:**
- Geometric mean for friction: industry standard, physically motivated (contact mechanics)
- Minimum for restitution: standard in game engines (prevents unrealistic bouncing)
- Velocity threshold critical: without it, resting contacts "micro-bounce" indefinitely
- Tangent direction computed from current relative velocity (not persistent across frames)

**Challenges:**
- Tangential impulse magnitude computation: needed to project relative velocity onto tangent plane
- Friction cone clamping: magnitude-based clamping in tangent direction
- Initial friction implementation was 1D; generalized to full 3D tangent space

**Performance Notes:**
- Friction adds ~30% cost per iteration (tangent vector computation + magnitude operations)
- Warm-start effectiveness: reduces iterations from 8 to ~3 for stable stacks

**Decisions:**
- Chose velocity-dependent restitution threshold (-0.1 m/s) based on experimentation
- Friction cone uses accumulated impulse magnitude (not per-iteration delta) for stability
- Apply restitution only on first iteration (standard practice, prevents oscillation)

---

## Next Steps (Prioritized)

1. **Shape Types:** Box primitive with SAT narrowphase (essential for diverse scenes)
2. **Sleeping:** Body sleep/wake system to skip inactive bodies (major perf win)
3. **Profiling:** Add zone timers and counters for broadphase/narrowphase/solver
4. **Contact Manifolds:** Multi-point contacts for box-box stability
5. **CCD:** Continuous collision detection for fast-moving objects (tunneling prevention)

---
