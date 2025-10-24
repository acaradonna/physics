#include "ape/ape.h"
#include <cassert>

int main(){
    ape::World w;
    ape::RigidBodyDesc a{}; a.position={0,0,0}; a.velocity={0,0,0}; a.mass=1.0f; a.sphere_radius=0.5f;
    ape::RigidBodyDesc b{}; b.position={0.6f,0,0}; b.velocity={0,0,0}; b.mass=1.0f; b.sphere_radius=0.5f;
    auto ida = w.createRigidBody(a);
    auto idb = w.createRigidBody(b);
    // Step a few frames; ensure relative normal velocity is non-negative (no interpenetration acceleration)
    for (int i=0;i<10;i++) w.step(1.0f/60.0f);
    auto va = w.getVelocity(ida);
    auto vb = w.getVelocity(idb);
    // Normal along +x direction expected
    float vn = (vb.x - va.x);
    assert(vn >= -1e-4f);
    return 0;
}


