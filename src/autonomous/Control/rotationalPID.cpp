#include "RotationalPID.hpp"
#include <cmath>
#include <algorithm>

RotationalPID::RotationalPID(double p, double i, double d, double tol,
                             double minOutput, double maxOutput)
    : kP(p), kI(i), kD(d),
      integral(0),
      lastError(0),
      outputMin(minOutput),
      outputMax(maxOutput),
      tolerance(tol) {}

void RotationalPID::rotateTo(double targetAngle) {
    integral = 0;
    lastError = 0;

    double startHeading = imu.get_rotation(); 
    double relativeTarget = startHeading + targetAngle;

    uint32_t prevTime = pros::millis();
    bool firstLoop = true;
    int settledCycles = 0;

    while (true) {
        double currentHeading = imu.get_rotation();

        double error = relativeTarget - currentHeading;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        uint32_t now = pros::millis();
        double dt = (now - prevTime) / 1000.0;
        prevTime = now;
        if (dt <= 0.001) dt = 0.01;

        double derivative = 0;
        if (!firstLoop) derivative = (error - lastError) / dt;
        firstLoop = false;

        integral += error * dt;
        integral = std::clamp(integral, -500.0, 500.0);

        double velocityOutput = kP * error + kI * integral + kD * derivative;

        if (error > 8 && velocityOutput < 50) velocityOutput = 50;
        if (error < -8 && velocityOutput > -50) velocityOutput = -50;

        velocityOutput = std::clamp(velocityOutput, outputMin, outputMax);

        left_mg.moveMotorsVelocity(-velocityOutput);
        right_mg.moveMotorsVelocity(velocityOutput);

        lastError = error;

        if (fabs(error) < tolerance) {
            settledCycles++;
        } else {
            settledCycles = 0;
        }

        if (settledCycles > 5) break;

        pros::delay(10);
    }

    left_mg.moveMotorsVelocity(0);
    right_mg.moveMotorsVelocity(0);
}
