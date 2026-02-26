#pragma once

#include "main.h"

class Odometry {
private:
    double x = 0;
    double y = 0;
    double theta = 0;

    double prevDistance = 0;
    double prevTheta = 0;

    static constexpr double wheelRadius = 1.75/2;
    static constexpr double ticksPerRev = 360.0;

    double horizontalDistance = 0;
    double verticalDistance = 0;

    static constexpr double alpha = 0.9;

    double deltaS = 0;
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
};

