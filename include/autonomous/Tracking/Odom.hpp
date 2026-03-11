#pragma once

#include "main.h"

class Odometry {
private:
    double x = 0;
    double y = 0;
    double theta = 0;

    double prevDistance = 0;
    double prevTheta = 0;

    static constexpr double wheelRadius = 1.75 / 2;
    static constexpr double ticksPerRev = 360.0;

    // Cached sensor readings in inches
    double frontDist_in = 0;
    double backDist_in  = 0;
    double leftDist_in  = 0;
    double rightDist_in = 0;

    // Physical offset from robot center to each sensor face (inches)
    static constexpr double frontOffset = 7.0;
    static constexpr double backOffset  = 7.0;
    static constexpr double leftOffset  = 7.0;
    static constexpr double rightOffset = 7.0;

    static constexpr double fieldSize = 144.0;
    static constexpr double alpha = 0.9;

    double deltaS     = 0;
    double deltaTheta = 0;

public:
    void init();
    void reset(double x = 0, double y = 0, double heading_deg = 0);
    void update();
    void resetDistance();

    double getX();
    double getY();
    double getHeadingRad();
    double getHeadingDeg();

    double getDeltaS();
    double getDeltaTheta();

    double getFrontDist();
    double getBackDist();
    double getLeftDist();
    double getRightDist();
};