#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include "ape/ape.h"
#include "ape/broadphase.h"

namespace ape {

struct Contact {
    std::uint32_t a;
    std::uint32_t b;
    float nx, ny, nz; // normal from a to b (unit length)
    float penetration; // amount to separate along normal
    float friction;    // combined friction coefficient for this pair
    float restitution; // combined restitution coefficient for this pair
};

// Generate sphere-sphere contacts from candidate pairs.
// Inputs:
// - positions: array of size count
// - radii: array of size count
// - frictions: array of size count
// - restitutions: array of size count
// - pairs: candidate pairs from broadphase
// Output:
// - out: appended with detected contacts
void generate_contacts_sphere_sphere(const Vec3* positions,
                                     const float* radii,
                                     const float* frictions,
                                     const float* restitutions,
                                     std::size_t count,
                                     const std::vector<Pair>& pairs,
                                     std::vector<Contact>& out);

// Unified narrowphase dispatcher handling all shape combinations
// Inputs:
// - positions, shape_types, sphere_radii, box_half_extents: per-body arrays
// - frictions, restitutions: material properties
// - pairs: candidate pairs from broadphase
// Output:
// - out: appended with detected contacts
void generate_contacts(const Vec3* positions,
                       const uint8_t* shape_types,
                       const float* sphere_radii,
                       const Vec3* box_half_extents,
                       const float* frictions,
                       const float* restitutions,
                       std::size_t count,
                       const std::vector<Pair>& pairs,
                       std::vector<Contact>& out);

} // namespace ape


