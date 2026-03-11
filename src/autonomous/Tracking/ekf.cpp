#include "main.h"

EKF::EKF() {
    std::memset(state.x, 0, sizeof(state.x));
    std::memset(state.P, 0, sizeof(state.P));

    for (int i = 0; i < STATE_DIM; i++) {
        state.P[i][i] = 0.01;
    }
}

void EKF::initialize(double x0, double y0, const Quaternion& q0) {
    state.x[0] = x0;
    state.x[1] = y0;
    state.x[2] = 0;
    state.x[3] = 0;

    state.x[4] = q0.w;
    state.x[5] = q0.x;
    state.x[6] = q0.y;
    state.x[7] = q0.z;
}

void EKF::predict(double vx, double vy, const Quaternion& dq, double dt) {
    state.x[0] += vx * dt;
    state.x[1] += vy * dt;

    Quaternion q(
        state.x[4],
        state.x[5],
        state.x[6],
        state.x[7]
    );

    q = dq * q;
    q.normalize();

    state.x[4] = q.w;
    state.x[5] = q.x;
    state.x[6] = q.y;
    state.x[7] = q.z;
}

void EKF::updatePosition(double measX, double measY, double R) {
    double yx = measX - state.x[0];
    double yy = measY - state.x[1];

    double K = state.P[0][0] / (state.P[0][0] + R);

    state.x[0] += K * yx;
    state.x[1] += K * yy;

    state.P[0][0] *= (1 - K);
    state.P[1][1] *= (1 - K);
}

EKFState EKF::getState() const {
    return state;
}

void EKF::normalizeQuaternion() {
    Quaternion q(
        state.x[4],
        state.x[5],
        state.x[6],
        state.x[7]
    );
    q.normalize();
    state.x[4] = q.w;
    state.x[5] = q.x;
    state.x[6] = q.y;
    state.x[7] = q.z;
}
