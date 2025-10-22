#include "ape/ape.h"
#include <vector>
#include <cmath>
#include <cstdint>
#include <limits>
#include "ape/broadphase.h"

namespace ape {

struct World::Impl {
    // Stable 32-bit handle: [high 16 bits generation][low 16 bits index]
    static constexpr uint32_t INDEX_BITS = 16;
    static constexpr uint32_t INDEX_MASK = (1u << INDEX_BITS) - 1u; // 0xFFFF

    // SoA storage
    std::vector<Vec3> pos;
    std::vector<Vec3> vel;
    std::vector<float> mass;
    std::vector<uint16_t> gen; // per-slot generation counters
    std::vector<uint16_t> alive; // 1 if occupied, 0 if free (small, cache-friendly)
    std::vector<uint16_t> free_list; // indices available for reuse

    Vec3 gravity{0.f, -9.80665f, 0.f};

    // Broadphase scratch and stats (temporary until proper pipeline)
    std::vector<AABB> aabbs;
    std::vector<Pair> pairs;
    uint32_t last_pair_count{0};

    static uint32_t pack_handle(uint16_t index, uint16_t generation) {
        return (static_cast<uint32_t>(generation) << INDEX_BITS) | static_cast<uint32_t>(index);
    }
    static uint16_t handle_index(uint32_t h) { return static_cast<uint16_t>(h & INDEX_MASK); }
    static uint16_t handle_generation(uint32_t h) { return static_cast<uint16_t>(h >> INDEX_BITS); }
};

World::World() : impl(new Impl) {}
World::~World() { delete impl; }

std::uint32_t World::createRigidBody(const RigidBodyDesc& d) {
    uint16_t idx;
    if (!impl->free_list.empty()) {
        idx = impl->free_list.back();
        impl->free_list.pop_back();
        // reuse slot; generation stays as is for valid new handle
        if (idx >= impl->pos.size()) {
            // should not happen; defensive
            idx = static_cast<uint16_t>(impl->pos.size());
            impl->pos.push_back(d.position);
            impl->vel.push_back(d.velocity);
            impl->mass.push_back(d.mass);
            impl->gen.push_back(0);
            impl->alive.push_back(1);
            return Impl::pack_handle(idx, impl->gen[idx]);
        }
        impl->pos[idx] = d.position;
        impl->vel[idx] = d.velocity;
        impl->mass[idx] = d.mass;
        impl->alive[idx] = 1;
    } else {
        if (impl->pos.size() >= std::numeric_limits<uint16_t>::max()) {
            // Out of indices; return invalid handle
            return std::numeric_limits<uint32_t>::max();
        }
        idx = static_cast<uint16_t>(impl->pos.size());
        impl->pos.push_back(d.position);
        impl->vel.push_back(d.velocity);
        impl->mass.push_back(d.mass);
        impl->gen.push_back(0);
        impl->alive.push_back(1);
    }
    return Impl::pack_handle(idx, impl->gen[idx]);
}

void World::step(float dt) {
    // Toy integrator with gravity to validate pipeline
    const Vec3 g = impl->gravity;
    const size_t n = impl->pos.size();
    for (size_t i = 0; i < n; ++i) {
        if (!impl->alive[i]) continue;
        Vec3 v = impl->vel[i];
        v.x += g.x * dt; v.y += g.y * dt; v.z += g.z * dt;
        Vec3 p = impl->pos[i];
        p.x += v.x * dt; p.y += v.y * dt; p.z += v.z * dt;
        impl->vel[i] = v;
        impl->pos[i] = p;
    }

    // Compute naive AABBs (sphere radius r = 0.5 for now)
    const float r = 0.5f;
    impl->aabbs.clear();
    impl->aabbs.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        if (!impl->alive[i]) { impl->aabbs.push_back(AABB{0,0,0,0,0,0}); continue; }
        const Vec3 p = impl->pos[i];
        AABB box{p.x - r, p.y - r, p.z - r, p.x + r, p.y + r, p.z + r};
        impl->aabbs.push_back(box);
    }
    impl->pairs.clear();
    broadphase_naive(impl->aabbs.data(), impl->aabbs.size(), impl->pairs);
    impl->last_pair_count = static_cast<uint32_t>(impl->pairs.size());
}

Vec3 World::getPosition(std::uint32_t id) const {
    const uint16_t idx = Impl::handle_index(id);
    const uint16_t g = Impl::handle_generation(id);
    if (idx >= impl->pos.size()) return Vec3{0,0,0};
    if (!impl->alive[idx]) return Vec3{0,0,0};
    if (impl->gen[idx] != g) return Vec3{0,0,0};
    return impl->pos[idx];
}

void World::setGravity(const Vec3& g) { impl->gravity = g; }
Vec3 World::getGravity() const { return impl->gravity; }

std::uint32_t World::debug_broadphasePairCount() const { return impl->last_pair_count; }

} // namespace ape
