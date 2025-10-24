# Research Notes

Physics algorithms, mathematical foundations, and design trade-offs.

Last updated: 2025-10-24 11:39 EDT

---

## Contact Resolution Methods

### Projection vs Impulse-Based

**Projection (Position-Based):**

- Directly modifies positions to eliminate overlap
- Simple, stable, no velocity consideration
- ❌ Non-physical (ignores momentum, mass)
- ✅ Guaranteed separation in one step
- Use case: Simple games, character controllers

**Impulse-Based (Velocity Constraints):**

- Modifies velocities via impulses; positions integrate naturally
- ✅ Physically accurate (conserves momentum)
- ✅ Handles friction, restitution, joints
- ❌ Requires iteration (PGS, SI, TGS)
- Use case: Realistic physics simulation

**Decision:** Hybrid approach - impulse-based PGS + optional projection for residual overlap.

---

## PGS (Projected Gauss-Seidel) Solver

### Algorithm Overview

Sequential impulse method solving velocity constraints:

```
for iteration in 1..N:
    for each contact:
        vn = dot(vb - va, normal)
        bias = baumgarte * (penetration / dt) [optional: + restitution]
        lambda = -(vn + bias) / (1/ma + 1/mb)
        clamp lambda >= 0  # no tension
        apply impulse to velocities
```

### Baumgarte Stabilization

**Formula:** `bias = β * (penetration / dt)`

**Purpose:** Add positional error correction to velocity constraint.

**Trade-offs:**

- β = 0: pure velocity constraint (objects sink slowly)
- β = 0.1-0.3: good balance (typical range)
- β > 0.5: over-corrects (jitter, instability)

**APE Choice:** β = 0.2 (empirically validated)

**References:**

- Baumgarte, J. (1972). "Stabilization of constraints and integrals of motion in dynamical systems"
- Catto, E. (2005). "Iterative Dynamics with Temporal Coherence"

---

## Warm-Starting

### Concept

Reuse accumulated impulses from previous frame as initial guess.

**Benefits:**

- Converges in 2-3 iterations vs 8+ cold-start
- Critical for stacks (prevents jitter)
- Cache locality benefits

**Implementation:**

- Store per-pair impulses with pair ID (a, b where a < b)
- Linear search to match pairs across frames (O(n²) but n small)
- Future: spatial hash for large contact counts

**APE Storage:**

```cpp
std::vector<Pair> warm_pairs;
std::vector<float> warm_impulses_n;   // normal
std::vector<Vec3> warm_impulses_t;    // tangent (friction)
```

---

## Material Property Combination

### Friction Combination

**Rule:** Geometric mean `μ = sqrt(μa * μb)`

**Justification:**

- Symmetric: `combine(a,b) == combine(b,a)`
- Physics: derived from Hertzian contact mechanics
- Edge cases: if either is 0, result is 0 (frictionless wins)

**Alternatives Considered:**

- Arithmetic mean: asymmetric at extremes
- Minimum: too conservative
- Maximum: unrealistic

**References:**

- Johnson, K.L. (1987). "Contact Mechanics"

### Restitution Combination

**Rule:** Minimum `e = min(ea, eb)`

**Justification:**

- Conservative: least bouncy material dominates
- Intuitive: ball on concrete vs ball on foam
- Prevents unrealistic energy gain

**Alternatives Considered:**

- Arithmetic mean: can create unexpected bounciness
- Product: too aggressive dampening
- Maximum: physically incorrect (energy creation)

**Standard Practice:** Box2D, Bullet, PhysX all use minimum.

---

## Coulomb Friction

### Friction Cone Constraint

**Formula:** `|Jt| <= μ * Jn`

**Meaning:** Tangential impulse limited by normal force and friction coefficient.

**Implementation Details:**

1. Compute tangent direction: `t = (rv - (rv·n)n) / |rv - (rv·n)n|`
2. Solve for tangential impulse: `deltaJt = -vt / k`
3. Accumulate and clamp: `Jt_new = clamp(Jt_old + deltaJt, -μ*Jn, μ*Jn)`

**Challenges:**

- Need to track tangent direction across iterations
- APE uses current velocity-based tangent (not persistent manifold tangent)
- Future: persistent tangent basis for better convergence

**References:**

- Catto, E. (2009). "Modeling and Solving Constraints"

---

## Restitution (Bounce)

### Velocity-Dependent Threshold

**Formula (first iteration only):**

```cpp
if (vn < -threshold && e > 0):
    bias -= e * vn
```

**Threshold Choice:** APE uses -0.1 m/s

**Justification:**

- Below threshold: treat as resting contact (no bounce)
- Above threshold: apply restitution
- Prevents micro-bouncing at rest
- Physics: real materials have velocity-dependent CoR

**Alternative Approaches:**

- Always apply restitution: causes jitter
- Energy-based threshold: more complex, similar results
- Hybrid restitution: blend between inelastic and elastic based on velocity

**References:**

- Stronge, W.J. (2000). "Impact Mechanics"

---

## Solver Iteration Count

### Trade-off Analysis

| Iterations | Convergence | Cost | Stability |
|------------|-------------|------|-----------|
| 1          | Poor        | 1x   | Unstable  |
| 4          | Fair        | 4x   | Usable    |
| 8          | Good        | 8x   | Stable    |
| 16         | Excellent   | 16x  | Overkill  |

**APE Choice:** 8 iterations (with warm-start)

**Rationale:**

- Stacks converge in 3-4 iterations with warm-start
- Marginal improvement beyond 8
- Industry standard: Box2D=8-10, Bullet=10, PhysX=4-8

**Future:** Adaptive iteration count based on convergence metric.

---

## Effective Mass (No Rotation)

**Formula:** `k = 1/ma + 1/mb`

**Current Limitation:** Point masses (no angular contribution)

**With Rotation (Future):**

```
k = 1/ma + 1/mb + (ra × n)ᵀ Ia⁻¹ (ra × n) + (rb × n)ᵀ Ib⁻¹ (rb × n)
```

Where:

- ra, rb: contact points relative to centers of mass
- Ia, Ib: inertia tensors
- n: contact normal

**Implementation Complexity:**

- Need inertia tensors per body
- Need contact point storage
- Need quaternion/matrix rotation state

---

## Next Research Topics

1. **Sequential Impulse (SI) vs PGS vs TGS:** Performance comparison
2. **XPBD:** Position-based dynamics with constraints (modern alternative to PGS)
3. **Contact Manifolds:** Multi-point contact generation and reduction
4. **SAT (Separating Axis Theorem):** For box-box collision detection
5. **Speculative Contacts:** Predict contacts before overlap (CCD alternative)
6. **Solver Preconditioning:** Reorder constraints for faster convergence

---

## References & Resources

### Papers

- Catto, E. (2005). "Iterative Dynamics with Temporal Coherence" - GDC
- Catto, E. (2009). "Modeling and Solving Constraints" - GDC
- Baumgarte, J. (1972). "Stabilization of constraints and integrals of motion"
- Stronge, W.J. (2000). "Impact Mechanics" - Cambridge

### Books

- Ericson, C. (2004). "Real-Time Collision Detection"
- Millington, I. (2007). "Game Physics Engine Development"
- Catto, E. & Hecker, C. "Game Physics" - GDC tutorials

### Open Source Engines (Study References)

- Box2D: 2D rigid body, excellent code quality
- Bullet: 3D rigid/soft body, feature-rich
- PhysX: Industry standard, well-documented
- Jolt: Modern C++, high-performance

---
