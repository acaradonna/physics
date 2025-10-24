#include "ape/ape.h"
#include <iostream>
int main(){
    ape::RigidBodyDesc d{};
    d.position = {0,1,0};
    d.velocity = {0,0,0};
    d.mass = 1.0f;
    d.sphere_radius = 0.5f;
    ape::World w;
    auto id = w.createRigidBody(d);
    for(int i=0;i<120;i++) w.step(1.0f/60.0f);
    auto p = w.getPosition(id);
    std::cout << p.y << "\n";
    return 0;
}
