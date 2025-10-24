#include "ape/broadphase.h"
#include <algorithm>

namespace ape
{

    void broadphase_naive(const AABB *boxes, std::size_t count, std::vector<Pair> &out)
    {
        out.clear();
        if (!boxes || count < 2)
            return;
        out.reserve(count); // heuristic; will grow as needed
        for (std::size_t i = 0; i + 1 < count; ++i)
        {
            for (std::size_t j = i + 1; j < count; ++j)
            {
                if (aabb_overlaps(boxes[i], boxes[j]))
                {
                    out.push_back(Pair{static_cast<std::uint32_t>(i), static_cast<std::uint32_t>(j)});
                }
            }
        }
    }

    void broadphase_sweep_1d(const AABB *boxes, std::size_t count, std::vector<Pair> &out, int axis)
    {
        out.clear();
        if (!boxes || count < 2) return;

        struct Endpoint { float value; std::uint32_t index; bool is_min; };
        std::vector<Endpoint> endpoints;
        endpoints.reserve(count * 2);

        for (std::size_t i = 0; i < count; ++i) {
            const AABB &b = boxes[i];
            float mn = axis == 0 ? b.min_x : (axis == 1 ? b.min_y : b.min_z);
            float mx = axis == 0 ? b.max_x : (axis == 1 ? b.max_y : b.max_z);
            endpoints.push_back(Endpoint{mn, static_cast<std::uint32_t>(i), true});
            endpoints.push_back(Endpoint{mx, static_cast<std::uint32_t>(i), false});
        }
        std::sort(endpoints.begin(), endpoints.end(), [](const Endpoint& a, const Endpoint& b){
            if (a.value < b.value) return true;
            if (a.value > b.value) return false;
            // Ensure starts before ends on tie to include touching as potential pairs
            return a.is_min && !b.is_min;
        });

        std::vector<std::uint32_t> active;
        active.reserve(count);
        for (const auto &e : endpoints) {
            if (e.is_min) {
                // Candidate pairs: e.index vs all active
                for (std::uint32_t j : active) {
                    const std::uint32_t a = std::min(e.index, j);
                    const std::uint32_t b = std::max(e.index, j);
                    if (aabb_overlaps(boxes[a], boxes[b])) {
                        out.push_back(Pair{a, b});
                    }
                }
                active.push_back(e.index);
            } else {
                // Remove from active
                auto it = std::find(active.begin(), active.end(), e.index);
                if (it != active.end()) {
                    *it = active.back();
                    active.pop_back();
                }
            }
        }
    }
} // namespace ape
