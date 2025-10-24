#include "ape/ape.h"
#include <cassert>

int main(){
    ape::World w;
    ape::RigidBodyDesc d{}; d.position = {0,1,0};
    std::uint32_t a = w.createRigidBody(d);
    std::uint32_t b = w.createRigidBody(d);
    assert(w.bodyCount() == 2);
    assert(w.isAlive(a));
    assert(w.isAlive(b));

    // Destroy one body; old handle should be invalidated
    w.destroyRigidBody(a);
    assert(!w.isAlive(a));
    assert(w.bodyCount() == 1);
    auto pa = w.getPosition(a);
    assert(pa.x == 0.0f && pa.y == 0.0f && pa.z == 0.0f);

    // Create a new body; count increases; old handle still dead
    std::uint32_t c = w.createRigidBody(d);
    assert(w.bodyCount() == 2);
    assert(!w.isAlive(a));
    assert(w.isAlive(b));
    assert(w.isAlive(c));

    // Destroy with mismatched generation should be a no-op
    // Use +2 to avoid matching the reused slot's current generation
    std::uint32_t bad_gen = ((a + (2u<<16)) & 0xFFFF0000u) | (a & 0xFFFFu);
    w.destroyRigidBody(bad_gen);
    assert(w.bodyCount() == 2);
    return 0;
}


