# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

- Track next planned changes here.

## 2025-10-24

- Add per-body `radius` to `RigidBodyDesc` and C ABI `ape_rigidbody_desc`.
- Implement sphere-sphere narrowphase and minimal positional solver.
- Update AABBs to respect per-body radius.
- Add tests: `collision_spheres`, update smoke and C API tests.
- Update build to include `src/collision/narrowphase.cpp`.
- Docs: mark narrowphase baseline done in `docs/ROADMAP.md`; update `ARCHITECTURE.md`, `PERFORMANCE-NOTES.md`.
- Web: add `Sphere Collision` demo to `web/index.html` with `web/js/demos/collision_spheres.js`.

## 2025-10-24 (later)

- Introduce frictionless PGS velocity solver with Baumgarte bias and warm-starting.
- Expose `getVelocity` in C++/C APIs; export in WASM and JS engine.
- Add `solver_velocity` test; wire into CMake.
- Update `docs/ROADMAP.md` marking PGS baseline complete.

## 2025-10-24 (even later)

- Add `friction` and `restitution` material properties to `RigidBodyDesc` and `Contact`.
- Implement Coulomb friction (tangential impulse solve) in PGS with friction cone clamping.
- Implement restitution (bounce) with velocity threshold to avoid jitter at rest.
- Update narrowphase to combine material properties (geometric mean for friction, min for restitution).
- Extend warm-start state to store both normal and tangential impulses.
- Add `restitution` test; bouncy ball validates positive velocity after bounce.
- Update `DEVLOG.md` with friction/restitution research notes.
