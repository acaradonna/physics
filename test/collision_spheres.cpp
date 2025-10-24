#include "ape/ape.h"
#include <cassert>
#include <cmath>

int main(){
    ape::World w;
    ape::RigidBodyDesc a{}; a.position={0,0,0}; a.velocity={0,0,0}; a.mass=1.0f; a.radius=0.5f;
    ape::RigidBodyDesc b{}; b.position={0.6f,0,0}; b.velocity={0,0,0}; b.mass=1.0f; b.radius=0.5f;
    auto ida = w.createRigidBody(a);
    auto idb = w.createRigidBody(b);
    // Step enough frames with positive dt to allow solver to resolve
    for (int i=0;i<60;i++) w.step(1.0f/60.0f);
    // After projection, distance between centers should be >= sum of radii (1.0)
    auto pa = w.getPosition(ida);
    auto pb = w.getPosition(idb);
    float dx = pb.x - pa.x;
    float dy = pb.y - pa.y;
    float dz = pb.z - pa.z;
    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    // Allow small tolerance for numerical error
    assert(dist + 1e-3f >= 1.0f);
    return 0;
}


