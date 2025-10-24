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
