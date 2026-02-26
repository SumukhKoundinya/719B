#pragma once

#include "main.h"

class LinearPID {
private:
    double kP, kI, kD;
    double integral;
    double lastError;
    double outputMin;
    double outputMax;
    double tolerance;

    const double ODOM_WHEEL_DIAMETER;
    const double ODOM_TICKS_PER_REV;

    double calculate(double targetDistance, double currentDistance, double dt);

public:
    LinearPID(double p, double i, double d,
               double wheelDiameter, double ticksPerRev,
               double tol = 0.5,
               double minOutput = -100, double maxOutput = 100);

    void moveDistance(double inches, double amplification);
    double getOdomDistance();
};
