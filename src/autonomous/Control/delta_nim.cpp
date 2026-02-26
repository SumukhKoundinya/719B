#include "main.h"

namespace DeltaNim {
    int32_t nimSum(const HeapVector& heaps) {
        return std::reduce(heaps.begin(), heaps.end(), 0, std::bit_xor{});
    }

    std::pair<int32_t, int32_t> sparseOptimalMove(const HeapVector& heaps) {
        int32_t current_xor = nimSum(heaps);
        if (current_xor == 0) return {-1, -1};

        for (size_t i = 0; i < heaps.size(); i++) {
            int32_t target = heaps[i] ^ current_xor;
            if (target < heaps[i] && target >= 0) {
                return {static_cast<int32_t>(i), heaps[i] - target};
            }
        }
        return {-1, -1};
    }

    std::pair<int32_t, int32_t> densePairingMove(const HeapVector& heaps) {
        auto max_it = std::max_element(heaps.begin(), heaps.end());
        if (*max_it > 0) {
            return {static_cast<int32_t>(std::distance(heaps.begin(), max_it)), 1};
        }
        return {-1, -1};
    }

    std::pair<int32_t, int32_t> optimalMove(const HeapVector& heaps) {
        int32_t support = 0;
        for (int32_t h: heaps) if (h > 0) support++;

        if (support <= SPARSE_CUTOFF) {
            return sparseOptimalMove(heaps);
        }
        return densePairingMove(heaps);
    }

    int32_t visionGoalLoad(int goal_idx) {
        static const int32_t right_alliance[4] = {3, 15, 0, 0};
        return right_alliance[goal_idx];
    }

    std::pair<double, double> getGoalPosition(int heap_idx) {
        const std::vector<std::pair<double, double>> GOALS = {
            {0,60},  // Long Goal Red
            {80,10}, // Long Goal Blue
            {52,30},  // Center Upper
            {-24,50}   // Center Lower
        };
        return GOALS[heap_idx % GOALS.size()];
    }

    HeapVector fieldToHeaps(double robot_x, double robot_y, double robot_theta) {
        HeapVector heaps(4, 0);

        for (int i = 0; i < 4; ++i) {
            auto [gx, gy] = getGoalPosition(i);
            double dist = hypot(robot_x - gx, robot_y - gy);
            double angle = atan2(gy - robot_y, gx - robot_x) - robot_theta;
            angle = fmod(angle + 3*M_PI, 2*M_PI) - M_PI;
            
            if (dist < 60.0 && fabs(angle) < M_PI/4.0) {
                heaps[i] = visionGoalLoad(i);
            }
        }
        return heaps;
    }
}
