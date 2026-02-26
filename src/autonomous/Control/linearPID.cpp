#include "main.h"

LinearPID::LinearPID(double p, double i, double d,
                       double wheelDiameter, double ticksPerRev,
                       double tol,
                       double minOutput, double maxOutput)
    : kP(p), kI(i), kD(d),
      integral(0), lastError(0),
      outputMin(minOutput), outputMax(maxOutput),
      tolerance(tol),
      ODOM_WHEEL_DIAMETER(wheelDiameter),
      ODOM_TICKS_PER_REV(ticksPerRev) {}

double LinearPID::calculate(double targetDistance, double currentDistance, double dt) {
    double error = targetDistance - currentDistance;
    integral += error * dt;

    const double integralLimit = 50;
    if (integral > integralLimit) integral = integralLimit;
    if (integral < -integralLimit) integral = -integralLimit;

    double derivative = (currentDistance - lastError) / dt;
    lastError = currentDistance;

    double output = kP * error + kI * integral + kD * derivative;

    output = std::clamp(output, outputMin, outputMax);

    if (output > 0 && output < 30) output = 30;
    if (output < 0 && output > -30) output = -30;

    return output;
}

double LinearPID::getOdomDistance() {
    int32_t sensorCentidegrees = vertPod.get_position();
    double sensorDegrees = sensorCentidegrees / 100.0;
    double rotations = sensorDegrees / 360.0;
    double distance = rotations * ODOM_WHEEL_DIAMETER * M_PI;
    distance = -distance;
    return distance;
}

void LinearPID::moveDistance(double inches, double amplification) {
    integral = 0;
    lastError = 0;

    double startX = pf.getX();
    double startY = pf.getY();
    double traveled = 0;

    int timeMs = 0;
    double lastMotorPower = 0;

    while (true) {
        double dx = pf.getX() - startX;
        double dy = pf.getY() - startY;
        traveled = sqrt(dx*dx + dy*dy);

        if (inches < 0) traveled = -traveled;

        double motorPower = calculate(inches, traveled, 0.02);

        motorPower = std::clamp(motorPower, -127.0, 127.0);

        if (motorPower > 0 && motorPower < 30) motorPower = 30;
        if (motorPower < 0 && motorPower > -30) motorPower = -30;

        double maxStep = 20;
        if (motorPower - lastMotorPower > maxStep) motorPower = lastMotorPower + maxStep;
        if (motorPower - lastMotorPower < -maxStep) motorPower = lastMotorPower - maxStep;
        lastMotorPower = motorPower;

        left_mg.moveMotors(motorPower * amplification);
        right_mg.moveMotors(motorPower * amplification);

        if (fabs(inches - traveled) < tolerance) break;

        lastError = traveled;
        timeMs += 20;
        if (timeMs > 3000) break;

        pros::delay(20);
    }

    left_mg.moveMotors(0);
    right_mg.moveMotors(0);
}
