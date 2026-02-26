#pragma once
#include "main.h"

class RotationalPID {
private:
    double kP;
    double kI;
    double kD;

    double integral;
    double lastError;

    double outputMin;
    double outputMax;

    double tolerance;

public:
    RotationalPID(double p, double i, double d,
                  double tol = 1.0,
                  double minOutput = -127.0,
                  double maxOutput = 127.0);

    void rotateTo(double targetAngle);
};
