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

    // Initialize all 4 sensor readings
    frontDist_in = vertFrontDist.get() / 25.4;
    backDist_in  = vertBackDist.get()  / 25.4;
    leftDist_in  = horizLeftDist.get()  / 25.4;
    rightDist_in = horizRightDist.get() / 25.4;
}

void Odometry::reset(double startX, double startY, double heading_deg) {
    x = startX;
    y = startY;
    theta = heading_deg * M_PI / 180.0;

    vertPod.reset_position();
    prevDistance = 0;
    prevTheta = theta;

    frontDist_in = vertFrontDist.get() / 25.4;
    backDist_in  = vertBackDist.get()  / 25.4;
    leftDist_in  = horizLeftDist.get()  / 25.4;
    rightDist_in = horizRightDist.get() / 25.4;
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

    // Cache latest sensor readings (converted to inches)
    frontDist_in = vertFrontDist.get() / 25.4;
    backDist_in  = vertBackDist.get()  / 25.4;
    leftDist_in  = horizLeftDist.get()  / 25.4;
    rightDist_in = horizRightDist.get() / 25.4;

    pros::lcd::print(0, "X: %.2f Y: %.2f T: %.2f", x, y, theta * 180.0 / M_PI);
}

void Odometry::resetDistance() {
    // Physical offsets: distance from robot center to each sensor face (inches)
    // Adjust these to match your actual sensor mounting positions
    const double frontOffset = 7.0;  // front sensor to robot center
    const double backOffset  = 7.0;  // back sensor to robot center
    const double leftOffset  = 7.0;  // left sensor to robot center
    const double rightOffset = 7.0;  // right sensor to robot center
    const double fieldSize   = 144.0;

    double cosT = cos(theta);
    double sinT = sin(theta);

    // --- X position: estimated from front and back sensors ---
    // Front sensor reads distance to the wall ahead; back sensor reads distance behind.
    // When theta=0, front points in +x direction.
    // x_from_front = fieldSize - frontReading - frontOffset (distance from origin wall)
    // x_from_back  = backReading + backOffset
    // Average them if both readings are valid (> 0 and < fieldSize)

    double xEstimates = 0;
    int xCount = 0;

    if (frontDist_in > 0 && frontDist_in < fieldSize) {
        xEstimates += (fieldSize - frontDist_in) * cosT - frontOffset * cosT;
        xCount++;
    }
    if (backDist_in > 0 && backDist_in < fieldSize) {
        xEstimates += backDist_in * cosT + backOffset * cosT;
        xCount++;
    }

    // --- Y position: estimated from left and right sensors ---
    if (leftDist_in > 0 && leftDist_in < fieldSize) {
        // Left sensor points in +y direction when theta=0
        // y = fieldSize - leftReading - leftOffset
        // But account for heading: project onto y-axis using sinT
        // Simplified for near-axis-aligned heading:
        xEstimates += 0; // placeholder structure — see note below
    }

    // Cleaner approach: handle X and Y separately with heading projection
    // X from front/back (these sensors measure along robot's forward axis)
    double newX = x, newY = y;

    if (frontDist_in > 0 && backDist_in > 0 &&
        frontDist_in < fieldSize && backDist_in < fieldSize) {
        // Front + back together span the full field along robot's forward axis
        // Total span should equal fieldSize minus both offsets
        // Robot center position along forward axis from origin:
        double span = frontDist_in + backDist_in + frontOffset + backOffset;
        // Ideally span == fieldSize; use it to compute position
        double posAlongForward = backDist_in + backOffset;
        // Project back to world X and Y
        newX = posAlongForward * cosT;
        newY = posAlongForward * sinT;
    } else if (frontDist_in > 0 && frontDist_in < fieldSize) {
        double posAlongForward = fieldSize - frontDist_in - frontOffset;
        newX = posAlongForward * cosT;
        newY = posAlongForward * sinT;
    } else if (backDist_in > 0 && backDist_in < fieldSize) {
        double posAlongForward = backDist_in + backOffset;
        newX = posAlongForward * cosT;
        newY = posAlongForward * sinT;
    }

    // Y correction from left/right sensors (perpendicular to heading)
    double sinT90 = sin(theta + M_PI / 2.0);  // left direction
    double cosT90 = cos(theta + M_PI / 2.0);

    if (leftDist_in > 0 && rightDist_in > 0 &&
        leftDist_in < fieldSize && rightDist_in < fieldSize) {
        double posAlongLeft = leftDist_in + leftOffset;
        newX += posAlongLeft * cosT90;
        newY += posAlongLeft * sinT90;
        // Average with forward estimate — weight equally
        newX /= 2.0;
        newY /= 2.0;
    } else if (leftDist_in > 0 && leftDist_in < fieldSize) {
        double posAlongLeft = leftDist_in + leftOffset;
        newX = (newX + posAlongLeft * cosT90) / 2.0;
        newY = (newY + posAlongLeft * sinT90) / 2.0;
    } else if (rightDist_in > 0 && rightDist_in < fieldSize) {
        double posAlongRight = rightDist_in + rightOffset;
        // Right sensor points in -y direction, so negate
        newX = (newX - posAlongRight * cosT90) / 2.0;
        newY = (newY - posAlongRight * sinT90) / 2.0;
    }

    x = newX;
    y = newY;

    vertPod.reset_position();
    prevDistance = 0;
    prevTheta = theta;

    pros::lcd::print(0, "Reset X: %.2f Y: %.2f T: %.2f", x, y, theta * 180.0 / M_PI);
    master.print(0, 0, "Reset X: %.2f", x);
    master.print(1, 0, "Reset Y: %.2f", y);
    master.print(2, 0, "Reset T: %.2f", theta * 180.0 / M_PI);
}

// Getters for the particle filter to consume
double Odometry::getFrontDist() { return frontDist_in; }
double Odometry::getBackDist()  { return backDist_in;  }
double Odometry::getLeftDist()  { return leftDist_in;  }
double Odometry::getRightDist() { return rightDist_in; }

double Odometry::getDeltaS()     { return deltaS; }
double Odometry::getDeltaTheta() { return deltaTheta; }
double Odometry::getX()          { return x; }
double Odometry::getY()          { return y; }
double Odometry::getHeadingRad() { return theta; }
double Odometry::getHeadingDeg() { return theta * 180.0 / M_PI; }