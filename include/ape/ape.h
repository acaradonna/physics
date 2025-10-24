#pragma once

// Public C++ API surface (evolving). Stable C ABI is in ape_c.h.
#include <cstdint>
#include <array>
#include <cstddef>

namespace ape {

struct Vec3 {
    float x, y, z;
};

struct RigidBodyDesc {
    Vec3 position{0,0,0};
    Vec3 velocity{0,0,0};
    float mass{1.0f};
    // Sphere radius for broadphase/narrowphase (temporary shape model)
    float radius{0.5f};
};

class World {
public:
    World();
    ~World();

    std::uint32_t createRigidBody(const RigidBodyDesc& desc);
    void destroyRigidBody(std::uint32_t id);
    void step(float dt);
    Vec3 getPosition(std::uint32_t id) const;
    Vec3 getVelocity(std::uint32_t id) const;

    // Global gravity (default 0,-9.80665,0)
    void setGravity(const Vec3& g);
    Vec3 getGravity() const;

    // Temporary debug helper: number of broadphase candidate pairs from last step
    std::uint32_t debug_broadphasePairCount() const;

    // Introspection helpers
    bool isAlive(std::uint32_t id) const;
    std::size_t bodyCount() const;

private:
    struct Impl;
    Impl* impl; // PIMPL for ABI stability
};

} // namespace ape
