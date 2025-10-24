#include "ape/ape_c.h"
#include <assert.h>

int main(){
    ape_world* w = ape_world_create();
    ape_rigidbody_desc d; d.position=(ape_vec3){0,0,0}; d.velocity=(ape_vec3){0,0,0}; d.mass=1.0f; d.radius=0.5f;
    uint32_t id = ape_world_create_rigidbody(w, d);
    assert(ape_world_is_alive(w, id) == 1u);
    ape_world_destroy_rigidbody(w, id);
    assert(ape_world_is_alive(w, id) == 0u);
    // Pointer variant
    uint32_t id2 = ape_world_create_rigidbody(w, d);
    ape_world_destroy_rigidbody_p(w, &id2);
    assert(ape_world_is_alive(w, id2) == 0u);
    ape_world_destroy(w);
    return 0;
}



