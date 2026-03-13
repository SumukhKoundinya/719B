#include "main.h"

LinearPID::LinearPID(double p, double i, double d,
                       double wheelDiameter, double ticksPerRev,
                       double tol,
                       double minOutput, double maxOutput, double kF)
    : kP(p), kI(i), kD(d),
      integral(0), lastError(0),
      outputMin(minOutput), outputMax(maxOutput),
      tolerance(tol),
      ODOM_WHEEL_DIAMETER(wheelDiameter),
      ODOM_TICKS_PER_REV(ticksPerRev),
      kF(kF) {}

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

std::vector<ProfilePoint> LinearPID::generateProfile(double distance, double maxVel, double maxAccel, double dt) {
    std::vector<ProfilePoint> profile;
    bool negative = distance < 0;
    distance = fabs(distance);

    double accelDist = (maxVel * maxVel) / (2.0 * maxAccel);
    if (2.0 * accelDist > distance) {
        maxVel = sqrt(maxAccel * distance);
        accelDist = distance / 2.0;
    }

    double cruiseDist = distance - 2.0 * accelDist;
    double pos = 0, vel = 0;

    while (vel < maxVel - 0.01) {
        vel = std::min(vel + maxAccel * dt, maxVel);
        pos += vel * dt;
        profile.push_back({negative ? -pos : pos, negative ? -vel : vel});
    }

    double cruiseTraveled = 0;
    while (cruiseTraveled < cruiseDist) {
        double step = std::min(maxVel * dt, cruiseDist - cruiseTraveled);
        pos += step;
        cruiseTraveled += step;
        profile.push_back({negative ? -pos : pos, negative ? -maxVel : maxVel});
    }

    while (vel > 0.01) {
        vel = std::max(vel - maxAccel * dt, 0.0);
        pos += vel * dt;
        profile.push_back({negative ? -pos : pos, negative ? -vel : vel});
    }

    return profile;
}

void LinearPID::moveDistanceProfiled(double inches, double maxVel, double maxAccel, double amplification) {
    integral = 0;
    lastError = 0;

    double startX = pf.getX();
    double startY = pf.getY();

    const double dt = 0.02;
    auto profile = generateProfile(inches, maxVel, maxAccel, dt);

    int timeMs = 0;

    for (auto& point : profile) {
        double dx = pf.getX() - startX;
        double dy = pf.getY() - startY;
        double traveled = sqrt(dx*dx + dy*dy);
        if (inches < 0) traveled = -traveled;

        double feedforward = kF * point.velocity;

        double pidOut = calculate(point.position, traveled, dt);

        double motorPower = std::clamp(feedforward + pidOut, -127.0, 127.0);

        left_mg.moveMotors(motorPower * amplification);
        right_mg.moveMotors(motorPower * amplification);

        lastError = traveled;;
        timeMs += 20;
        if (timeMs > 5000) break;

        pros::delay(20);
    }

    int settleMs = 0;
    while (settleMs < 500) {
        double dx = pf.getX() - startX;
        double dy = pf.getY() - startY;
        double traveled = sqrt(dx*dx + dy*dy);
        if (inches < 0) traveled = -traveled;

        double motorPower = calculate(inches, traveled, dt);
        motorPower = std::clamp(motorPower, -127.0, 127.0);

        left_mg.moveMotors(motorPower * amplification);
        right_mg.moveMotors(motorPower * amplification);

        if (fabs(inches - traveled) < tolerance) break;

        lastError = traveled;
        settleMs += 20;
        pros::delay(20);
    }

    left_mg.moveMotors(0);
    right_mg.moveMotors(0);
}
