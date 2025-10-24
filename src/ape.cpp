#include "ape/ape.h"
#include <vector>
#include <cmath>
#include <cstdint>
#include <limits>
#include "ape/broadphase.h"
#include "ape/narrowphase.h"

namespace ape {

struct World::Impl {
    // Stable 32-bit handle: [high 16 bits generation][low 16 bits index]
    static constexpr uint32_t INDEX_BITS = 16;
    static constexpr uint32_t INDEX_MASK = (1u << INDEX_BITS) - 1u; // 0xFFFF

    // SoA storage
    std::vector<Vec3> pos;
    std::vector<Vec3> vel;
    std::vector<float> mass;
    std::vector<float> radius;
    std::vector<uint16_t> gen; // per-slot generation counters
    std::vector<uint16_t> alive; // 1 if occupied, 0 if free (small, cache-friendly)
    std::vector<uint16_t> free_list; // indices available for reuse

    Vec3 gravity{0.f, -9.80665f, 0.f};

    // Broadphase scratch and stats (temporary until proper pipeline)
    std::vector<AABB> aabbs;
    std::vector<Pair> pairs;
    std::vector<Contact> contacts;
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
            impl->radius.push_back(d.radius > 0.0f ? d.radius : 0.5f);
            impl->gen.push_back(0);
            impl->alive.push_back(1);
            return Impl::pack_handle(idx, impl->gen[idx]);
        }
        impl->pos[idx] = d.position;
        impl->vel[idx] = d.velocity;
        impl->mass[idx] = d.mass;
        // ensure radius vector is large enough
        if (idx >= impl->radius.size()) impl->radius.resize(idx+1, 0.5f);
        impl->radius[idx] = d.radius > 0.0f ? d.radius : 0.5f;
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
        impl->radius.push_back(d.radius > 0.0f ? d.radius : 0.5f);
        impl->gen.push_back(0);
        impl->alive.push_back(1);
    }
    return Impl::pack_handle(idx, impl->gen[idx]);
}

void World::destroyRigidBody(std::uint32_t id) {
    const uint16_t idx = Impl::handle_index(id);
    const uint16_t g = Impl::handle_generation(id);
    if (idx >= impl->pos.size()) return;
    if (!impl->alive[idx]) return;
    if (impl->gen[idx] != g) return;
    impl->alive[idx] = 0;
    // Bump generation to invalidate outstanding handles
    impl->gen[idx] = static_cast<uint16_t>(impl->gen[idx] + 1);
    impl->free_list.push_back(idx);
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

    // Compute AABBs using per-body sphere radius (default 0.5f)
    impl->aabbs.clear();
    impl->aabbs.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        if (!impl->alive[i]) { impl->aabbs.push_back(AABB{0,0,0,0,0,0}); continue; }
        const Vec3 p = impl->pos[i];
        const float r = (i < impl->radius.size() && impl->radius[i] > 0.0f) ? impl->radius[i] : 0.5f;
        AABB box{p.x - r, p.y - r, p.z - r, p.x + r, p.y + r, p.z + r};
        impl->aabbs.push_back(box);
    }
    impl->pairs.clear();
    // Use sweep-and-prune along X as a baseline; future: choose best axis per frame
    broadphase_sweep_1d(impl->aabbs.data(), impl->aabbs.size(), impl->pairs, 0);
    impl->last_pair_count = static_cast<uint32_t>(impl->pairs.size());

    // Narrowphase: sphere-sphere contacts
    impl->contacts.clear();
    const float* radii_ptr = impl->radius.empty() ? nullptr : impl->radius.data();
    generate_contacts_sphere_sphere(impl->pos.data(), radii_ptr, impl->pos.size(), impl->pairs, impl->contacts);

    // Minimal solver: positional projection splitting equally along normal
    for (const Contact &c : impl->contacts) {
        const uint32_t ia = c.a;
        const uint32_t ib = c.b;
        if (ia >= impl->pos.size() || ib >= impl->pos.size()) continue;
        if (!impl->alive[ia] || !impl->alive[ib]) continue;
        const float half = 0.5f * c.penetration;
        Vec3 pa = impl->pos[ia];
        Vec3 pb = impl->pos[ib];
        pa.x -= c.nx * half; pa.y -= c.ny * half; pa.z -= c.nz * half;
        pb.x += c.nx * half; pb.y += c.ny * half; pb.z += c.nz * half;
        impl->pos[ia] = pa;
        impl->pos[ib] = pb;
    }
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

bool World::isAlive(std::uint32_t id) const {
    const uint16_t idx = Impl::handle_index(id);
    const uint16_t g = Impl::handle_generation(id);
    if (idx >= impl->pos.size()) return false;
    if (!impl->alive[idx]) return false;
    return impl->gen[idx] == g;
}

std::size_t World::bodyCount() const {
    std::size_t count = 0;
    const std::size_t n = impl->alive.size();
    for (std::size_t i = 0; i < n; ++i) if (impl->alive[i]) ++count;
    return count;
}

} // namespace ape
