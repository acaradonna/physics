#include "ape/broadphase.h"
#include <cassert>
#include <vector>
#include <algorithm>

static bool contains_pair(const std::vector<ape::Pair>& v, ape::Pair p){
    for (auto &q : v) if (q.a==p.a && q.b==p.b) return true; return false;
}

int main(){
    using namespace ape;
    // Mix of overlapping and non-overlapping boxes
    std::vector<AABB> boxes = {
        {0,0,0, 1,1,1},
        {0.5f,0.5f,0.5f, 1.5f,1.5f,1.5f}, // overlaps 0
        {3,3,3, 4,4,4},                   // separate
        {-1,-1,-1, -0.2f,-0.2f,-0.2f}     // separate
    };
    std::vector<Pair> naive;
    std::vector<Pair> sweep;
    broadphase_naive(boxes.data(), boxes.size(), naive);
    broadphase_sweep_1d(boxes.data(), boxes.size(), sweep, 0);
    // All sweep pairs must be in naive set
    for (auto &p : sweep) assert(contains_pair(naive, p));
    // At least the obvious overlap must be found
    assert(contains_pair(sweep, Pair{0,1}));
    return 0;
}



