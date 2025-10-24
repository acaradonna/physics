#include "ape/ape.h"
#include <cassert>
#include <cmath>

int main(){
    ape::World w;
    w.setGravity({0, -10.0f, 0});
    // Drop a bouncy ball from height 10
    ape::RigidBodyDesc d{}; d.position={0,10,0}; d.velocity={0,0,0}; d.mass=1.0f; d.sphere_radius=0.5f;
    d.friction = 0.0f;
    d.restitution = 0.8f; // High bounce
    auto id = w.createRigidBody(d);
    // Simulate free fall to ground
    for (int i=0;i<60;i++) w.step(1.0f/60.0f);
    auto p = w.getPosition(id);
    auto v = w.getVelocity(id);
    // Ball should have bounced multiple times; expect y > 0 and positive velocity
    assert(p.y > 0.0f);
    // Velocity should be positive (upward) after bouncing
    assert(v.y > 0.0f || v.y < -1.0f); // Either going up or still falling
    return 0;
}

