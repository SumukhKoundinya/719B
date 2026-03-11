#include "main.h"
#include <cmath>
#include <algorithm>

PurePursuit::PurePursuit(const std::vector<PathPoint>& p, double lookahead, double width) {
    path = p;
    lookaheadDistance = lookahead;
    trackWidth = width;
    lastIndex = 0;
    finished = false;
    lastLeft = 0;
    lastRight = 0;
    maxSlewStep = 15;
    minOutput = 25;
}

void PurePursuit::reset() {
    lastIndex = 0;
    finished = false;
    lastLeft = 0;
    lastRight = 0;
}

bool PurePursuit::isFinished() const { return finished; }

std::vector<PathPoint> loadPath(const char* filename) {
    return {
        {-62.434, -14.693, 54.729, 0, false},
        {-60.5,   -15.202, 49.872, 0, false},
        {-58.566, -15.711, 44.489, 0, false},
        {-56.632, -16.22,  38.357, 0, false},
        {-54.698, -16.729, 31.036, 0, false},
        {-52.763, -17.238, 21.337, 0, false},
        {-51.03,  -17.694, 0,      0, true},
    };
}

static int findClosest(double x, double y, const std::vector<PathPoint>& path) {
    int closest = 0;
    double closestDist = 1e9;
    for (int i = 0; i < (int)path.size(); i++) {
        double dx = path[i].x - x;
        double dy = path[i].y - y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist < closestDist) {
            closestDist = dist;
            closest = i;
        }
    }
    return closest;
}

static double findIntersect(PathPoint p1, PathPoint p2, double cx, double cy, double r) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double fx = p1.x - cx;
    double fy = p1.y - cy;

    double a = dx*dx + dy*dy;
    double b = 2*(fx*dx + fy*dy);
    double c = fx*fx + fy*fy - r*r;
    double disc = b*b - 4*a*c;

    if (disc < 0) return -1;

    double t2 = (-b + sqrt(disc)) / (2*a);
    double t1 = (-b - sqrt(disc)) / (2*a);

    if (t2 >= 0 && t2 <= 1) return t2;
    if (t1 >= 0 && t1 <= 1) return t1;
    return -1;
}

void PurePursuit::step(const Pose& pose) {
    if (finished || path.empty()) return;

    int closest = findClosest(pose.x, pose.y, path);

    if (path[closest].speed == 0) {
        setMotors(0, 0);
        finished = true;
        return;
    }

    double lookaheadX = path.back().x;
    double lookaheadY = path.back().y;
    int lookaheadIndex = (int)path.size() - 1;

    int start = std::max(closest, lastIndex);
    for (int i = start; i < (int)path.size() - 1; i++) {
        double t = findIntersect(path[i], path[i+1], pose.x, pose.y, lookaheadDistance);
        if (t != -1) {
            lookaheadX = path[i].x + t * (path[i+1].x - path[i].x);
            lookaheadY = path[i].y + t * (path[i+1].y - path[i].y);
            lookaheadIndex = i;
            break;
        }
    }
    lastIndex = lookaheadIndex;

    double dx = lookaheadX - pose.x;
    double dy = lookaheadY - pose.y;

    double localX =  cos(pose.theta) * dx + sin(pose.theta) * dy;
    double localY = -sin(pose.theta) * dx + cos(pose.theta) * dy;

    double L = sqrt(localX*localX + localY*localY);
    if (L < 1e-6) return;

    double curvature = (2.0 * localY) / (L * L);

    double targetSpeed = path[closest].speed;

    double leftVel  = targetSpeed * (2 + curvature * trackWidth) / 2;
    double rightVel = targetSpeed * (2 - curvature * trackWidth) / 2;

    double ratio = std::max(fabs(leftVel), fabs(rightVel)) / 127.0;
    if (ratio > 1) {
        leftVel  /= ratio;
        rightVel /= ratio;
    }

    if (leftVel  - lastLeft  >  maxSlewStep) leftVel  = lastLeft  + maxSlewStep;
    if (leftVel  - lastLeft  < -maxSlewStep) leftVel  = lastLeft  - maxSlewStep;
    if (rightVel - lastRight >  maxSlewStep) rightVel = lastRight + maxSlewStep;
    if (rightVel - lastRight < -maxSlewStep) rightVel = lastRight - maxSlewStep;

    lastLeft  = leftVel;
    lastRight = rightVel;

    setMotors(leftVel, rightVel);
}

void PurePursuit::setMotors(double left, double right) {
    left_mg.moveMotors(left);
    right_mg.moveMotors(right);
}