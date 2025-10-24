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
    uint32_t last_pair_count{0};
    std::vector<Contact> contacts;
    // Solver warm-start state
    std::vector<Pair> warm_pairs;
    std::vector<float> warm_impulses;   // per-pair accumulated normal impulse
    std::vector<float> solver_impulses; // current-frame accumulated normal impulse

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
    const Vec3 g = impl->gravity;
    const size_t n = impl->pos.size();
    // 1) Integrate velocities with gravity
    for (size_t i = 0; i < n; ++i) {
        if (!impl->alive[i]) continue;
        Vec3 v = impl->vel[i];
        v.x += g.x * dt; v.y += g.y * dt; v.z += g.z * dt;
        impl->vel[i] = v;
    }

    // 2) Broadphase on current positions
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
    broadphase_sweep_1d(impl->aabbs.data(), impl->aabbs.size(), impl->pairs, 0);
    impl->last_pair_count = static_cast<uint32_t>(impl->pairs.size());

    // 3) Narrowphase contacts
    impl->contacts.clear();
    const float* radii_ptr = impl->radius.empty() ? nullptr : impl->radius.data();
    generate_contacts_sphere_sphere(impl->pos.data(), radii_ptr, impl->pos.size(), impl->pairs, impl->contacts);

    // 4) Frictionless PGS solver with warm-start
    const int solverIterations = 8;
    const float baumgarte = 0.2f; // positional error correction factor

    // Initialize current impulses and warm-start
    impl->solver_impulses.assign(impl->contacts.size(), 0.0f);
    if (!impl->warm_pairs.empty() && !impl->warm_impulses.empty()) {
        for (size_t i = 0; i < impl->contacts.size(); ++i) {
            const Pair key{impl->contacts[i].a, impl->contacts[i].b};
            for (size_t j = 0; j < impl->warm_pairs.size(); ++j) {
                if (impl->warm_pairs[j].a == key.a && impl->warm_pairs[j].b == key.b) {
                    const float J = impl->warm_impulses[j];
                    impl->solver_impulses[i] = J;
                    // Apply warm-start impulse to velocities
                    const float invMa = (impl->mass[key.a] > 0.0f) ? (1.0f / impl->mass[key.a]) : 0.0f;
                    const float invMb = (impl->mass[key.b] > 0.0f) ? (1.0f / impl->mass[key.b]) : 0.0f;
                    Vec3 va = impl->vel[key.a];
                    Vec3 vb = impl->vel[key.b];
                    const Contact &c = impl->contacts[i];
                    va.x -= c.nx * (J * invMa); va.y -= c.ny * (J * invMa); va.z -= c.nz * (J * invMa);
                    vb.x += c.nx * (J * invMb); vb.y += c.ny * (J * invMb); vb.z += c.nz * (J * invMb);
                    impl->vel[key.a] = va; impl->vel[key.b] = vb;
                    break;
                }
            }
        }
    }

    if (dt > 0.0f) {
        for (int iter = 0; iter < solverIterations; ++iter) {
            for (size_t i = 0; i < impl->contacts.size(); ++i) {
                const Contact &c = impl->contacts[i];
                const uint32_t ia = c.a, ib = c.b;
                if (!impl->alive[ia] || !impl->alive[ib]) continue;
                const float invMa = (impl->mass[ia] > 0.0f) ? (1.0f / impl->mass[ia]) : 0.0f;
                const float invMb = (impl->mass[ib] > 0.0f) ? (1.0f / impl->mass[ib]) : 0.0f;
                if (invMa == 0.0f && invMb == 0.0f) continue;
                Vec3 va = impl->vel[ia];
                Vec3 vb = impl->vel[ib];
                const float rvx = vb.x - va.x;
                const float rvy = vb.y - va.y;
                const float rvz = vb.z - va.z;
                const float vn = rvx * c.nx + rvy * c.ny + rvz * c.nz;
                const float bias = baumgarte * (c.penetration / dt);
                const float k = invMa + invMb;
                if (k <= 0.0f) continue;
                float deltaJ = -(vn + bias) / k;
                float J = impl->solver_impulses[i];
                float Jnew = J + deltaJ;
                if (Jnew < 0.0f) Jnew = 0.0f;
                deltaJ = Jnew - J;
                if (deltaJ != 0.0f) {
                    va.x -= c.nx * (deltaJ * invMa); va.y -= c.ny * (deltaJ * invMa); va.z -= c.nz * (deltaJ * invMa);
                    vb.x += c.nx * (deltaJ * invMb); vb.y += c.ny * (deltaJ * invMb); vb.z += c.nz * (deltaJ * invMb);
                    impl->vel[ia] = va; impl->vel[ib] = vb;
                    impl->solver_impulses[i] = Jnew;
                }
            }
        }
    }

    // Optional positional correction (post-solve) to eliminate residual overlap
    if (!impl->contacts.empty()) {
        const float slop = 1e-4f;     // allow tiny overlap to avoid jitter
        const float percent = 0.8f;   // resolve most of the penetration
        for (const Contact &c : impl->contacts) {
            const uint32_t ia = c.a, ib = c.b;
            if (!impl->alive[ia] || !impl->alive[ib]) continue;
            Vec3 pa = impl->pos[ia];
            Vec3 pb = impl->pos[ib];
            // Recompute separation along the stored normal
            const float dx = pb.x - pa.x;
            const float dy = pb.y - pa.y;
            const float dz = pb.z - pa.z;
            const float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            // Target separation is encoded in c.penetration as the original overlap
            // Use that as a guide; correct residual if still overlapping
            const float residual = c.penetration - (dist < 1e-12f ? 0.0f : 0.0f);
            (void)residual; // not used directly; rely on current dist vs desired
            // Compute penetration from current positions: desired is no overlap
            // Project along normal if still overlapping (distance smaller than prior radii sum)
            // c.penetration was computed as (ra+rb - dist_at_narrowphase)
            // We approximate residual using c.penetration when dist hasn't changed much.
            float corr = c.penetration - slop;
            if (corr > 0.0f) {
                corr *= percent * 0.5f;
                pa.x -= c.nx * corr; pa.y -= c.ny * corr; pa.z -= c.nz * corr;
                pb.x += c.nx * corr; pb.y += c.ny * corr; pb.z += c.nz * corr;
                impl->pos[ia] = pa;
                impl->pos[ib] = pb;
            }
        }
    }

    // Save warm-start for next frame
    impl->warm_pairs.clear();
    impl->warm_impulses.clear();
    impl->warm_pairs.reserve(impl->contacts.size());
    impl->warm_impulses.reserve(impl->contacts.size());
    for (size_t i = 0; i < impl->contacts.size(); ++i) {
        impl->warm_pairs.push_back(Pair{impl->contacts[i].a, impl->contacts[i].b});
        impl->warm_impulses.push_back(impl->solver_impulses[i]);
    }

    // 5) Integrate positions with solved velocities
    for (size_t i = 0; i < n; ++i) {
        if (!impl->alive[i]) continue;
        Vec3 p = impl->pos[i];
        const Vec3 v = impl->vel[i];
        p.x += v.x * dt; p.y += v.y * dt; p.z += v.z * dt;
        impl->pos[i] = p;
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

Vec3 World::getVelocity(std::uint32_t id) const {
    const uint16_t idx = Impl::handle_index(id);
    const uint16_t g = Impl::handle_generation(id);
    if (idx >= impl->vel.size()) return Vec3{0,0,0};
    if (!impl->alive[idx]) return Vec3{0,0,0};
    if (impl->gen[idx] != g) return Vec3{0,0,0};
    return impl->vel[idx];
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
