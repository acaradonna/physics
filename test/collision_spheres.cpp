#include "ape/ape.h"
#include <cassert>

int main(){
    ape::World w;
    ape::RigidBodyDesc a{}; a.position={0,0,0}; a.velocity={0,0,0}; a.mass=1.0f; a.radius=0.5f;
    ape::RigidBodyDesc b{}; b.position={0.6f,0,0}; b.velocity={0,0,0}; b.mass=1.0f; b.radius=0.5f;
    auto ida = w.createRigidBody(a);
    auto idb = w.createRigidBody(b);
    w.step(0.0f);
    // After projection, distance between centers should be >= sum of radii (1.0)
    auto pa = w.getPosition(ida);
    auto pb = w.getPosition(idb);
    float dx = pb.x - pa.x;
    float dy = pb.y - pa.y;
    float dz = pb.z - pa.z;
    float dist2 = dx*dx + dy*dy + dz*dz;
    assert(dist2 + 1e-5f >= 1.0f);
    return 0;
}


