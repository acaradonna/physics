# Architecture (APE)

Goal: Maximize performance, realism, and extensibility across platforms and languages.

Key principles

- Data-Oriented Design: SoA layouts, cache-friendly pipelines, SIMD and task-based parallelism.
- Determinism: Fixed-step core, stable reductions, optional replay/rollback.
- Modularity: Rigid, soft, cloth, fluids, particles as pluggable subsystems.
- Stable C ABI: Minimal surface, versioned, zero-copy views where safe.
- Precision tiers: fast-float (float32) default, double and mixed precision opt-in.

Core subsystems

- Foundation: math, allocators, jobs, profiling, logging.
- Broadphase: BVH/Sweep-And-Prune with incremental update, GPU path later.
  - Baseline implemented: 1D sweep-and-prune along X with stable candidate ordering and 3D AABB filter.
- Narrowphase: sphere-sphere baseline implemented; GJK/EPA, SAT, and CCD planned.
- Constraints: iterative Gauss-Seidel/PGS with warm starting; explore XPBD.
- Integrators: semi-implicit Euler baseline; RK2/Verlet and symplectic options.
- Materials: friction, restitution, anisotropy; contact models.
- Scene/World: islands, sleeping, deterministic ordering.
  - Handles: 32-bit stable handles `[generation:16][index:16]`; free-list reuse with generation bump on destroy.
  - Lifecycle: create/destroy, `isAlive`, `bodyCount` for introspection.

Threading model

- Work-stealing job system with affinities; lock-free queues.
- Task graph per step: update -> broadphase -> narrowphase -> solve -> integrate.

Memory

- Arena + slab + scratch allocators, per-frame transient buffers.
- SoA pools for bodies, shapes, joints; handles not raw pointers.

Extensibility

- Plugin API for new shapes, joints, solvers.
- Event hooks (pre/post step, contact callbacks) via C ABI.

Interop & targets

- Native: Windows/Linux/macOS (x86_64, ARM64).
- Mobile: iOS/Android.
- Web: WebAssembly + SIMD + threads (SharedArrayBuffer) where available.
- Bindings: C, Python (pybind11/C API), Node (N-API), C# (P/Invoke), Java (JNI), Rust (bindgen).

Determinism notes

- Avoid non-deterministic reductions; use fixed traversal and pair ordering.
- Explicit random seeds where needed.

Observability

- Built-in lightweight profiler zones, counters; compile-time toggles.

Security/safety

- No UB; sanitize, fuzz; robust input validation on C ABI.

Versioning

- Semantic versioning; C ABI version gates.
