## APE (Alpackle Physics Engine) - Working Roadmap

This file tracks near-term tasks between agent sessions. For long-term phases, see `docs/ROADMAP.md`.

### Active sprint (next 1-2 weeks)

- Core: finalize body lifecycle (done: destroy/isAlive/bodyCount)
- Broadphase: add Sweep-And-Prune (1D) baseline and tests
- Determinism: add unit tests for fixed step ordering and pair ordering
- Observability: add lightweight profiling counters and compile-time toggle
- CI: enable AddressSanitizer/UBSan in Debug matrix and basic fuzz target

### Backlog (prioritized)

- Narrowphase: GJK+EPA baseline for sphere/box/capsule
- Constraints: PGS solver skeleton with warm start buffers
- Shapes: sphere and box primitives with AABB generation
- Bindings: Python pybind11 alpha exposing `World` and basic APIs
- Web: Emscripten build target smoke test in `web/`
- Docs: API reference sections for C and C++ surfaces

### Nice-to-have

- Job system: work stealing and per-thread scratch arenas
- Benchmarks: microbenchmarks for SoA access and broadphase scaling

### Notes

- Keep C ABI stable; prefer adding new functions over changing signatures.
- Maintain deterministic ordering in pair generation (a<b, stable traversal).

