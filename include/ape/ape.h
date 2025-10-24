#pragma once

// Public C++ API surface (evolving). Stable C ABI is in ape_c.h.
#include <cstdint>
#include <array>
#include <cstddef>

namespace ape {

struct Vec3 {
    float x, y, z;
};

enum class ShapeType : std::uint8_t {
    Sphere = 0,
    Box = 1,
};

struct RigidBodyDesc {
    Vec3 position{0,0,0};
    Vec3 velocity{0,0,0};
    float mass{1.0f};
    
    // Shape (shape_type determines which shape fields are used)
    ShapeType shape_type{ShapeType::Sphere};
    float sphere_radius{0.5f};        // used when shape_type == Sphere
    Vec3 box_half_extents{0.5f,0.5f,0.5f}; // used when shape_type == Box (half-width, half-height, half-depth)
    
    // Material properties
    float friction{0.5f};    // Coulomb friction coefficient (0=frictionless, 1=high friction)
    float restitution{0.0f}; // Coefficient of restitution (0=inelastic, 1=perfectly elastic)
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
