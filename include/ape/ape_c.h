#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

// Minimal C ABI to enable early bindings and smoke tests

typedef struct ape_vec3 { float x, y, z; } ape_vec3;

typedef struct ape_rigidbody_desc {
    ape_vec3 position;
    ape_vec3 velocity;
    float mass;
    // Sphere radius (temporary baseline shape); ignored if <= 0
    float radius;
} ape_rigidbody_desc;

typedef struct ape_world ape_world; // opaque

// Versioning
uint32_t ape_version_major(void);
uint32_t ape_version_minor(void);
uint32_t ape_version_patch(void);

ape_world* ape_world_create(void);
void ape_world_destroy(ape_world* w);

uint32_t ape_world_create_rigidbody(ape_world* w, ape_rigidbody_desc desc);
void ape_world_destroy_rigidbody(ape_world* w, uint32_t id);
void ape_world_step(ape_world* w, float dt);
ape_vec3 ape_world_get_position(const ape_world* w, uint32_t id);

void ape_world_set_gravity(ape_world* w, ape_vec3 g);
ape_vec3 ape_world_get_gravity(const ape_world* w);

// Pointer-based variants for WASM/FFI friendliness (no struct-by-value crossings)
uint32_t ape_world_create_rigidbody_p(ape_world* w, const ape_rigidbody_desc* desc);
void ape_world_destroy_rigidbody_p(ape_world* w, const uint32_t* id);
void ape_world_get_position_out(const ape_world* w, uint32_t id, ape_vec3* out);
void ape_world_set_gravity_p(ape_world* w, const ape_vec3* g);
void ape_world_get_gravity_out(const ape_world* w, ape_vec3* out);

// Introspection helpers
uint32_t ape_world_is_alive(const ape_world* w, uint32_t id); // 0 false, 1 true
size_t ape_world_body_count(const ape_world* w);

#ifdef __cplusplus
}
#endif
