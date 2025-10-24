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
};

// Generate sphere-sphere contacts from candidate pairs.
// Inputs:
// - positions: array of size count
// - radii: array of size count
// - pairs: candidate pairs from broadphase
// Output:
// - out: appended with detected contacts
void generate_contacts_sphere_sphere(const Vec3* positions,
                                     const float* radii,
                                     std::size_t count,
                                     const std::vector<Pair>& pairs,
                                     std::vector<Contact>& out);

} // namespace ape


