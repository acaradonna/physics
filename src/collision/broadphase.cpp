#include "ape/broadphase.h"

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

} // namespace ape
