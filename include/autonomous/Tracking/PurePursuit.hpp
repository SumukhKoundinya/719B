#pragma once
#include "main.h"
#include <vector>
#include <string>

struct Pose {
    double x;
    double y;
    double theta;
};

struct PathPoint {
    double x;
    double y;
    double speed;
    double heading;
    bool hasHeading;
};

class PurePursuit {
private:
    std::vector<PathPoint> path;

    double lookaheadDistance;
    double trackWidth;

    int lastIndex;
    bool finished;

    double lastLeft;
    double lastRight;

    double maxSlewStep;
    double minOutput;

public:
    PurePursuit(const std::vector<PathPoint>& p,
                double lookahead,
                double width);

    void step(const Pose& pose);

    bool isFinished() const;

    void reset();

    void setMotors(double left, double right);

    std::vector<PathPoint> getPath() const { return path; }
};

std::vector<PathPoint> loadPath(const char* filename);