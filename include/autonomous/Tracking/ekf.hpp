#pragma once
#include "main.h"
#include "quaternion.hpp" 

constexpr int STATE_DIM = 8;

struct EKFState {
    double x[STATE_DIM];
    double P[STATE_DIM][STATE_DIM];
};

class EKF {
public:
    EKF();

    void initialize(double x0, double y0, const Quaternion& q0);

    void predict(double vx, double vy, const Quaternion& dq, double dt);

    void updatePosition(double measX, double measY, double R);

    EKFState getState() const;

private:
    EKFState state;

    void normalizeQuaternion();
};
