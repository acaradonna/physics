#include "ape/narrowphase.h"
#include <cmath>

namespace ape {

static inline float rsqrt_safe(float x) {
    if (x <= 0.0f) return 0.0f;
    return 1.0f / std::sqrt(x);
}

void generate_contacts_sphere_sphere(const Vec3* positions,
                                     const float* radii,
                                     const float* frictions,
                                     const float* restitutions,
                                     std::size_t /*count*/,
                                     const std::vector<Pair>& pairs,
                                     std::vector<Contact>& out)
{
    if (!positions || !radii) return;
    for (const Pair &p : pairs) {
        const Vec3 pa = positions[p.a];
        const Vec3 pb = positions[p.b];
        const float ra = radii[p.a] > 0.0f ? radii[p.a] : 0.5f;
        const float rb = radii[p.b] > 0.0f ? radii[p.b] : 0.5f;
        const float dx = pb.x - pa.x;
        const float dy = pb.y - pa.y;
        const float dz = pb.z - pa.z;
        const float dist2 = dx*dx + dy*dy + dz*dz;
        const float rsum = ra + rb;
        if (dist2 < rsum * rsum) {
            float nx = 0.0f, ny = 0.0f, nz = 0.0f;
            float penetration = 0.0f;
            if (dist2 > 1e-12f) {
                const float inv_dist = rsqrt_safe(dist2);
                nx = dx * inv_dist; ny = dy * inv_dist; nz = dz * inv_dist;
                const float dist = 1.0f / inv_dist;
                penetration = rsum - dist;
            } else {
                // centers coincide; choose an arbitrary normal (y-up)
                nx = 0.0f; ny = 1.0f; nz = 0.0f;
                penetration = rsum;
            }
            // Combine material properties: geometric mean for friction, min for restitution
            const float fa = frictions ? frictions[p.a] : 0.5f;
            const float fb = frictions ? frictions[p.b] : 0.5f;
            const float combined_friction = std::sqrt(fa * fb);
            const float ra_rest = restitutions ? restitutions[p.a] : 0.0f;
            const float rb_rest = restitutions ? restitutions[p.b] : 0.0f;
            const float combined_restitution = (ra_rest < rb_rest) ? ra_rest : rb_rest;
            out.push_back(Contact{p.a, p.b, nx, ny, nz, penetration, combined_friction, combined_restitution});
        }
    }
}

} // namespace ape


