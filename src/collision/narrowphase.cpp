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

// Helper: clamp point to AABB
static inline Vec3 clamp_to_aabb(const Vec3& point, const Vec3& center, const Vec3& half_extents) {
    Vec3 result;
    result.x = (point.x < center.x - half_extents.x) ? (center.x - half_extents.x) : 
               (point.x > center.x + half_extents.x) ? (center.x + half_extents.x) : point.x;
    result.y = (point.y < center.y - half_extents.y) ? (center.y - half_extents.y) : 
               (point.y > center.y + half_extents.y) ? (center.y + half_extents.y) : point.y;
    result.z = (point.z < center.z - half_extents.z) ? (center.z - half_extents.z) : 
               (point.z > center.z + half_extents.z) ? (center.z + half_extents.z) : point.z;
    return result;
}

// Sphere-Box collision using closest-point-on-box method
static bool collide_sphere_box(const Vec3& sphere_center, float sphere_radius,
                                const Vec3& box_center, const Vec3& box_half_extents,
                                Vec3& normal_out, float& penetration_out) {
    // Find closest point on box surface to sphere center
    Vec3 closest;
    closest.x = std::max(box_center.x - box_half_extents.x, std::min(sphere_center.x, box_center.x + box_half_extents.x));
    closest.y = std::max(box_center.y - box_half_extents.y, std::min(sphere_center.y, box_center.y + box_half_extents.y));
    closest.z = std::max(box_center.z - box_half_extents.z, std::min(sphere_center.z, box_center.z + box_half_extents.z));
    
    const float dx = sphere_center.x - closest.x;
    const float dy = sphere_center.y - closest.y;
    const float dz = sphere_center.z - closest.z;
    const float dist2 = dx*dx + dy*dy + dz*dz;
    
    if (dist2 < sphere_radius * sphere_radius) {
        const float dist = std::sqrt(dist2);
        if (dist > 1e-6f) {
            // Normal points from closest point to sphere center
            normal_out.x = dx / dist;
            normal_out.y = dy / dist;
            normal_out.z = dz / dist;
            penetration_out = sphere_radius - dist;
        } else {
            // Sphere center inside box; choose normal based on minimal separation
            const float dx_dist = box_half_extents.x - std::abs(sphere_center.x - box_center.x);
            const float dy_dist = box_half_extents.y - std::abs(sphere_center.y - box_center.y);
            const float dz_dist = box_half_extents.z - std::abs(sphere_center.z - box_center.z);
            if (dx_dist < dy_dist && dx_dist < dz_dist) {
                normal_out = Vec3{(sphere_center.x < box_center.x) ? -1.0f : 1.0f, 0, 0};
                penetration_out = sphere_radius + dx_dist;
            } else if (dy_dist < dz_dist) {
                normal_out = Vec3{0, (sphere_center.y < box_center.y) ? -1.0f : 1.0f, 0};
                penetration_out = sphere_radius + dy_dist;
            } else {
                normal_out = Vec3{0, 0, (sphere_center.z < box_center.z) ? -1.0f : 1.0f};
                penetration_out = sphere_radius + dz_dist;
            }
        }
        return true;
    }
    return false;
}

// Box-Box collision using SAT (axis-aligned only for now; rotation in future)
static bool collide_box_box(const Vec3& ca, const Vec3& ha,
                             const Vec3& cb, const Vec3& hb,
                             Vec3& normal_out, float& penetration_out) {
    // Compute overlap on each axis
    const float dx = std::abs(cb.x - ca.x);
    const float dy = std::abs(cb.y - ca.y);
    const float dz = std::abs(cb.z - ca.z);
    const float overlap_x = (ha.x + hb.x) - dx;
    const float overlap_y = (ha.y + hb.y) - dy;
    const float overlap_z = (ha.z + hb.z) - dz;
    
    // Check separation on each axis
    if (overlap_x <= 0.0f || overlap_y <= 0.0f || overlap_z <= 0.0f) {
        return false; // Separated
    }
    
    // Find axis of minimum penetration
    float min_overlap = overlap_x;
    int axis = 0;
    if (overlap_y < min_overlap) { min_overlap = overlap_y; axis = 1; }
    if (overlap_z < min_overlap) { min_overlap = overlap_z; axis = 2; }
    
    penetration_out = min_overlap;
    
    // Normal points from a to b along minimum penetration axis
    if (axis == 0) {
        normal_out = Vec3{(cb.x > ca.x) ? 1.0f : -1.0f, 0, 0};
    } else if (axis == 1) {
        normal_out = Vec3{0, (cb.y > ca.y) ? 1.0f : -1.0f, 0};
    } else {
        normal_out = Vec3{0, 0, (cb.z > ca.z) ? 1.0f : -1.0f};
    }
    
    return true;
}

// Unified narrowphase dispatcher
void generate_contacts(const Vec3* positions,
                       const uint8_t* shape_types,
                       const float* sphere_radii,
                       const Vec3* box_half_extents,
                       const float* frictions,
                       const float* restitutions,
                       std::size_t count,
                       const std::vector<Pair>& pairs,
                       std::vector<Contact>& out)
{
    if (!positions || !shape_types) return;
    
    for (const Pair &p : pairs) {
        const ShapeType type_a = static_cast<ShapeType>(shape_types[p.a]);
        const ShapeType type_b = static_cast<ShapeType>(shape_types[p.b]);
        
        Vec3 normal{0,0,0};
        float penetration = 0.0f;
        bool colliding = false;
        
        const Vec3 pa = positions[p.a];
        const Vec3 pb = positions[p.b];
        
        if (type_a == ShapeType::Sphere && type_b == ShapeType::Sphere) {
            // Sphere-Sphere
            const float ra = sphere_radii ? sphere_radii[p.a] : 0.5f;
            const float rb = sphere_radii ? sphere_radii[p.b] : 0.5f;
            const float dx = pb.x - pa.x;
            const float dy = pb.y - pa.y;
            const float dz = pb.z - pa.z;
            const float dist2 = dx*dx + dy*dy + dz*dz;
            const float rsum = ra + rb;
            if (dist2 < rsum * rsum) {
                const float dist = std::sqrt(dist2);
                if (dist > 1e-12f) {
                    normal.x = dx / dist; normal.y = dy / dist; normal.z = dz / dist;
                    penetration = rsum - dist;
                } else {
                    normal = Vec3{0, 1, 0};
                    penetration = rsum;
                }
                colliding = true;
            }
        } else if ((type_a == ShapeType::Sphere && type_b == ShapeType::Box) ||
                   (type_a == ShapeType::Box && type_b == ShapeType::Sphere)) {
            // Sphere-Box
            const bool a_is_sphere = (type_a == ShapeType::Sphere);
            const Vec3 sphere_center = a_is_sphere ? pa : pb;
            const Vec3 box_center = a_is_sphere ? pb : pa;
            const float radius = (sphere_radii && a_is_sphere) ? sphere_radii[p.a] : 
                                 (sphere_radii && !a_is_sphere) ? sphere_radii[p.b] : 0.5f;
            const Vec3 half_ext = (box_half_extents && !a_is_sphere) ? box_half_extents[p.a] :
                                  (box_half_extents && a_is_sphere) ? box_half_extents[p.b] : Vec3{0.5f,0.5f,0.5f};
            
            colliding = collide_sphere_box(sphere_center, radius, box_center, half_ext, normal, penetration);
            // collide_sphere_box returns normal FROM box TO sphere (separation direction)
            // This is what we want regardless of which body is 'a' - it separates them
        } else if (type_a == ShapeType::Box && type_b == ShapeType::Box) {
            // Box-Box (axis-aligned)
            const Vec3 ha = box_half_extents ? box_half_extents[p.a] : Vec3{0.5f,0.5f,0.5f};
            const Vec3 hb = box_half_extents ? box_half_extents[p.b] : Vec3{0.5f,0.5f,0.5f};
            colliding = collide_box_box(pa, ha, pb, hb, normal, penetration);
        }
        
        if (colliding) {
            // Combine material properties
            const float fa = frictions ? frictions[p.a] : 0.5f;
            const float fb = frictions ? frictions[p.b] : 0.5f;
            const float combined_friction = std::sqrt(fa * fb);
            const float ra = restitutions ? restitutions[p.a] : 0.0f;
            const float rb = restitutions ? restitutions[p.b] : 0.0f;
            const float combined_restitution = (ra < rb) ? ra : rb;
            out.push_back(Contact{p.a, p.b, normal.x, normal.y, normal.z, penetration, combined_friction, combined_restitution});
        }
    }
}

} // namespace ape


