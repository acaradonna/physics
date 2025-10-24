#include "ape/ape.h"
#include <cassert>
#include <iostream>

int main() {
    ape::World world;
    ape::RigidBodyDesc d{};
    d.position = {0, 10, 0};
    d.mass = 1.0f;
    d.radius = 0.5f;
    auto id = world.createRigidBody(d);

    for (int i=0;i<60;i++) world.step(1.0f/60.0f);
    auto p = world.getPosition(id);

    std::cout << "y=" << p.y << "\n";
    assert(p.y < 10.0f);
    return 0;
}
