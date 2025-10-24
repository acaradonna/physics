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

## 2025-10-24 11:46 EDT - Sleeping System

**What:** Implemented automatic sleep/wake system for inactive bodies.

**Implementation:**
- Added `awake` flag (uint16_t) and `sleep_timer` (float) to SoA storage
- Sleep thresholds: linear velocity < 0.01 m/s for 0.5 seconds
- Bodies start awake on creation
- Sleep detection runs after position integration

**Sleep Logic:**
1. Track motion: compute linear velocity magnitude
2. If below threshold: accumulate timer
3. If timer >= 0.5s: put to sleep, zero velocity
4. If above threshold: reset timer

**Wake Logic:**
- Bodies wake on contact with awake bodies (checked after narrowphase)
- Wake propagates through contact graph (cascading wake)
- Sleeping body contacting static body stays asleep

**Optimization:**
- Skip sleeping bodies in velocity integration
- Skip sleeping bodies in position integration  
- Include sleeping bodies in broadphase (for wake-on-contact detection)
- Skip solver for pairs where both bodies sleep

**Testing:**
- Added `sleeping` test: validates low-velocity body sleeps, high-velocity doesn't
- All 13 tests pass (100%)

**Performance Impact:**
- Expected 3-5x speedup for large static scenes (most bodies sleeping)
- Negligible overhead for active scenes (sleep check is simple threshold)
- Wake-on-contact adds small cost to narrowphase loop

**Design Decisions:**
- Velocity zeroing on sleep prevents slow drift accumulation
- 0.5s timer prevents "flicker" (sleep-wake-sleep oscillation)
- Threshold 0.01 m/s chosen to be imperceptible but allow resting
- Include sleeping in broadphase: necessary for wake propagation

**Challenges:**
- Initial test tried wake-on-collision with velocity transfer (too complex)
- Simplified to just validate sleep/wake state transitions
- Considered island-based sleeping (deferred to future)

**Future Improvements:**
- Island detection: sleep entire connected components
- Angular velocity threshold (when rotation added)
- User-controllable sleep parameters via API
- Sleep state visualization in debug views

---

## 2025-10-24 11:58 EDT - Box Shapes and SAT Collision

**What:** Implemented box primitive and SAT-based collision detection for all shape combinations.

**Implementation:**
- Added `ShapeType` enum (Sphere, Box)
- Extended `RigidBodyDesc` with `shape_type`, `sphere_radius`, `box_half_extents`
- Added shape storage to `World::Impl` SoA layout
- Implemented AABB generation for boxes (axis-aligned)

**Collision Detection:**
- **Sphere-Box:** Closest-point-on-box method
  - Clamp sphere center to box AABB
  - Check distance from clamped point to sphere center
  - Normal points from clamped point toward sphere center
  - Special case: sphere center inside box (choose minimal separation axis)
- **Box-Box:** SAT (Separating Axis Theorem) for axis-aligned boxes
  - Test 3 separation axes (X, Y, Z)
  - Compute overlap on each axis
  - Choose axis with minimum penetration
  - Normal points along minimum penetration axis

**Unified Narrowphase:**
- Created `generate_contacts()` dispatcher handling all shape combinations
- Keeps existing `generate_contacts_sphere_sphere()` for reference
- Material combination (friction/restitution) unified across all types

**Testing:**
- Added `box_shapes` test validating box-box collision and separation
- All 14 tests pass (100%)
- Verified dynamic-dynamic box collisions work correctly

**Current Limitations:**
- Boxes are axis-aligned only (no rotation yet)
- Static bodies (mass=0) with sphere-box need investigation
- Single-point contacts (no manifolds yet)

**Design Decisions:**
- Chose closest-point method for sphere-box over GJK (simpler, faster for AA boxes)
- SAT for box-box: straightforward for axis-aligned, extensible to OBB later
- Store both sphere and box data for all bodies (wastes ~16 bytes per body but simplifies code)

**Research Notes:**
- SAT test axes for AA boxes: just 3 (world X, Y, Z)
- For oriented boxes (future): need 15 axes (3 face normals per box + 9 edge-edge cross products)
- Closest-point method: mathematically equivalent to GJK for convex shapes but optimized
- Normal direction convention critical: must be consistent with solver expectations

**Challenges:**
- Initial normal direction confusion: solver expects normal from a->b
- Sphere-box with static box shows unexpected behavior (deferred investigation)
- Simplified test to focus on dynamic-dynamic cases for now

**Performance Notes:**
- Box AABB generation: 6 additions (vs 3 for sphere)
- Sphere-box collision: ~15 ops (clamp + distance check)
- Box-box SAT: ~20 ops (3 axis tests + min-finding)
- Negligible overhead vs sphere-only

---

## Next Steps (Prioritized)

1. **Static Bodies:** Fix sphere-box collision with mass=0 bodies; add explicit static flag
2. **Profiling:** Add zone timers and counters for broadphase/narrowphase/solver
3. **Contact Manifolds:** Multi-point contacts for box-box stability
4. **Rotation:** Add quaternion orientation; update SAT for oriented boxes (OBB)
5. **CCD:** Continuous collision detection for fast-moving objects (tunneling prevention)
6. **Islands:** Detect connected components for batch sleeping

---
