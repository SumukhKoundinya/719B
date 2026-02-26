#include "main.h"

void Odometry::init() {
    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(10);
    }

    vertPod.reset_position();
    prevDistance = 0;

    theta = imu.get_rotation() * M_PI / 180.0;
    prevTheta = theta;

    horizontalDistance = horizDist.get() / 25.4;
    verticalDistance = vertDist.get() / 25.4;
}

void Odometry::reset(double startX, double startY, double heading_deg) {
    x = startX;
    y = startY;
    theta = heading_deg * M_PI / 180.0;

    vertPod.reset_position();
    prevDistance = 0;
    prevTheta = theta;

    horizontalDistance = horizDist.get() / 25.4;
    verticalDistance = vertDist.get() / 25.4;
}

void Odometry::update() {
    double currentTicks = vertPod.get_position();
    double sensorDegrees = currentTicks / 100.0;
    double rotations = sensorDegrees / ticksPerRev;
    double currentDistance = rotations * 2 * wheelRadius * M_PI;
    currentDistance = -currentDistance;

    deltaS = currentDistance - prevDistance;
    prevDistance = currentDistance;

    double imuDegrees = imu.get_rotation();
    theta = imuDegrees * M_PI / 180.0;

    deltaTheta = theta - prevTheta;
    double thetaMid = prevTheta + deltaTheta / 2.0;
    prevTheta = theta;

    x += deltaS * cos(thetaMid);
    y += deltaS * sin(thetaMid);

    pros::lcd::print(0, "X: %.2f Y: %.2f T: %.2f", x, y, theta * 180.0 / M_PI);
}

void Odometry::resetDistance() {
    double horizReading = horizDist.get() / 25.4;
    double vertReading = vertDist.get() / 25.4;

    double horizOffset = 3.0;
    double vertOffset = 2.5;

    double cosTheta = cos(theta);
    double sinTheta = sin(theta);

    if (horizReading > 0) {
        x = horizReading - horizOffset * cosTheta;
    }

    if (vertReading > 0) {
        y = vertReading - vertOffset * sinTheta;
    }

    vertPod.reset_position();
    prevDistance = 0;
    prevTheta = theta;

    horizontalDistance = horizReading;
    verticalDistance = vertReading;

    pros::lcd::print(0, "Reset X: %.2f Y: %.2f T: %.2f", x, y, theta * 180.0 / M_PI);
    master.print(0, 0, "Reset X: %.2f", x);
    master.print(1, 0, "Reset Y: %.2f", y);
    master.print(2, 0, "Reset T: %.2f", theta * 180.0 / M_PI);
}

double Odometry::getDeltaS() {
    return deltaS;
}

double Odometry::getDeltaTheta() {
    return deltaTheta;
}

double Odometry::getX() { return x; }
double Odometry::getY() { return y; }
double Odometry::getHeadingRad() { return theta; }
double Odometry::getHeadingDeg() { return theta * 180.0 / M_PI; }
