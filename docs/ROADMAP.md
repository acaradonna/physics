# Roadmap

Phase 0: Incubation (this repo)

- [x] Minimal C++ core skeleton + C ABI
- [x] CMake build + smoke test
- [x] Job system foundation (threads, queues)
- [x] SoA body storage + handles
- [x] Body lifecycle: destroy, generation bump, isAlive, bodyCount
- [x] Broadphase (AABB, SAP baseline)
- [x] Narrowphase (sphere-sphere baseline)
- [x] Minimal positional solver (projection) for penetration resolution
- [x] Iterative solver (PGS) with warm-start (frictionless, baseline)
- [ ] Deterministic step guarantees + tests
- [ ] Profiling + logging infra
- [ ] Fuzz tests, sanitizer CI

Phase 1: Rigid body MVP

- [ ] Basic shapes (sphere, box, capsule)
- [ ] Continuous collision detection (swept volumes)
- [ ] Joints (distance, hinge, fixed)
- [ ] Sleep/awake, island management
- [ ] Python and C# bindings alpha

Phase 2: Advanced features

- [ ] Mesh collisions, convex decomposition
- [ ] XPBD constraints, cloth and soft bodies
- [ ] Fluids (APIC/FLIP) experiments
- [ ] GPU broadphase experiments
- [ ] WebAssembly build with threads + SIMD

Phase 3: Production hardening

- [ ] Deterministic replay/rollback API
- [ ] Extensive benchmarks suite
- [ ] Documentation site + examples gallery
