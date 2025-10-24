#pragma once

// Minimal broadphase scaffolding (AABB + naive O(n^2) overlap finder)
// Future work: incremental SAP/BVH; this is for early tests only.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ape {

struct AABB {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
};

inline bool aabb_overlaps(const AABB& a, const AABB& b) {
    return (a.min_x <= b.max_x && a.max_x >= b.min_x) &&
           (a.min_y <= b.max_y && a.max_y >= b.min_y) &&
           (a.min_z <= b.max_z && a.max_z >= b.min_z);
}

struct Pair { std::uint32_t a, b; };

// Naive all-pairs broadphase; returns pairs with a < b
void broadphase_naive(const AABB* boxes, std::size_t count, std::vector<Pair>& out);

// Sweep-and-prune (1D) broadphase along the specified axis (0=x,1=y,2=z).
// Returns candidate pairs with a < b, filtered by full 3D AABB overlap.
void broadphase_sweep_1d(const AABB* boxes, std::size_t count, std::vector<Pair>& out, int axis = 0);

} // namespace ape
