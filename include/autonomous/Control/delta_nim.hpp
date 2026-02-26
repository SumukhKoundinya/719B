#pragma once

#include "main.h"

namespace DeltaNim {
    using HeapVector = std::vector<int32_t>;
    constexpr int32_t SPARSE_CUTOFF = 60;

    int32_t nimSum(const HeapVector& heaps);
    std::pair<int32_t, int32_t> optimalMove(const HeapVector& heaps);
    HeapVector fieldToHeaps(double robot_x, double robot_y, double robot_theta);
    std::pair<double, double> getGoalPosition(int heap_idx);
}
