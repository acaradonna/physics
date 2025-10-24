#include "ape/ape.h"
#include <cassert>

int main(){
    ape::World w;
    w.setGravity({0, 0, 0}); // No gravity to allow sleep

    // Test 1: Body with low velocity should fall asleep
    ape::RigidBodyDesc d{};
    d.position = {0, 0, 0};
    d.velocity = {0.005f, 0, 0}; // Very low velocity (below sleep threshold 0.01)
    d.mass = 1.0f;
    d.sphere_radius = 0.5f;
    auto id = w.createRigidBody(d);

    // Step for 1 second (should accumulate sleep timer and sleep after 0.5s)
    for (int i = 0; i < 120; i++) { // 120 steps at 1/120 = 1 second
        w.step(1.0f/120.0f);
    }

    // After 1 second with low motion, body should have fallen asleep
    // Verify velocity is zero (sleep system zeros velocity)
    auto v = w.getVelocity(id);
    assert(v.x == 0.0f && v.y == 0.0f && v.z == 0.0f);

    // Test 2: Body with high velocity should NOT sleep
    ape::RigidBodyDesc d2{};
    d2.position = {5, 0, 0};
    d2.velocity = {1.0f, 0, 0}; // High velocity (above sleep threshold)
    d2.mass = 1.0f;
    d2.sphere_radius = 0.5f;
    auto id2 = w.createRigidBody(d2);

    // Step for 1 second
    for (int i = 0; i < 120; i++) {
        w.step(1.0f/120.0f);
    }

    // Body should still be moving (not asleep)
    v = w.getVelocity(id2);
    assert(v.x > 0.9f); // Should have nearly full velocity (no friction/drag in vacuum)

    return 0;
}

