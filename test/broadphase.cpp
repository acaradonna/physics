#include "ape/broadphase.h"
#include <cassert>
#include <vector>

int main(){
    using namespace ape;
    // Three boxes: 0 overlaps 1, 2 separate
    AABB boxes[3] = {
        {0,0,0, 1,1,1},
        {0.5f,0.5f,0.5f, 2,2,2},
        {3,3,3, 4,4,4}
    };
    std::vector<Pair> pairs;
    broadphase_naive(boxes, 3, pairs);
    assert(pairs.size() == 1);
    assert(pairs[0].a == 0 && pairs[0].b == 1);
    return 0;
}
