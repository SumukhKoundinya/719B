#pragma once

#include "main.h"

namespace DeltaNim {
    using HeapVector = std::vector<int32_t>;
    constexpr int32_t SPARSE_CUTOFF = 60;
    constexpr uint32_t PRESET_TIME = 3000;

    int32_t nimSum(const HeapVector& heaps);
    std::pair<int32_t, int32_t> optimalMove(const HeapVector& heaps);
    HeapVector fieldToHeaps(double robot_x, double robot_y, double robot_theta, 
                        double enemy_x = 0, double enemy_y = 0, bool enemy_blocked = false);
        std::pair<double, double> getGoalPosition(int heap_idx);
};