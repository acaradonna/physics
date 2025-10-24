# Language bindings plan

Goals

- Keep C ABI tiny and stable. Prefer opaque handles and flat structs.
- Zero-copy where safe; explicit lifetimes.
- Avoid per-call allocation and locks.

Targets and notes

- Python: pybind11 or CPython C API with midspeed path; numpy views for SoA buffers.
- Node.js: N-API; expose minimal classes; use worker threads for stepping.
- C#: P/Invoke + source generators; Span\<T\> over pinned buffers when possible.
- Java: JNI thin layer; direct byte buffers for arrays.
- Rust: bindgen + safe wrapper; no panics across FFI.
- Web: Emscripten + WASM SIMD/threads; transferable memory for data snapshots.

Versioning & features

- ABI version in `ape_c.h`; query at runtime.
- Feature flags for optional subsystems.

Current C ABI surface

- World lifecycle: `ape_world_create`, `ape_world_destroy`
- Bodies: `ape_world_create_rigidbody`, `ape_world_destroy_rigidbody`, pointer variant `_p`
- Queries: `ape_world_get_position`, pointer out variant, `ape_world_is_alive`, `ape_world_body_count`
- Globals: `ape_world_set_gravity`, `ape_world_get_gravity` and pointer variants
