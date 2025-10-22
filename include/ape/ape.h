#pragma once

// Public C++ API surface (evolving). Stable C ABI is in ape_c.h.
#include <cstdint>
#include <array>

namespace ape {

struct Vec3 {
    float x, y, z;
};

struct RigidBodyDesc {
    Vec3 position{0,0,0};
    Vec3 velocity{0,0,0};
    float mass{1.0f};
};

class World {
public:
    World();
    ~World();

    std::uint32_t createRigidBody(const RigidBodyDesc& desc);
    void step(float dt);
    Vec3 getPosition(std::uint32_t id) const;

    // Global gravity (default 0,-9.80665,0)
    void setGravity(const Vec3& g);
    Vec3 getGravity() const;

    // Temporary debug helper: number of broadphase candidate pairs from last step
    std::uint32_t debug_broadphasePairCount() const;

private:
    struct Impl;
    Impl* impl; // PIMPL for ABI stability
};

} // namespace ape
