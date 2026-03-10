#pragma once

#include <cmath>

struct Quaternion {
    double w, x, y, z;
    
    Quaternion();
    Quaternion(double w, double x, double y, double z);

    static Quaternion fromHeading(double theta);
    static Quaternion identity();

    double toHeading() const;

    Quaternion operator*(const Quaternion &rhs) const;
    Quaternion conjugate() const;
    double normSq() const;
    double norm() const;
    Quaternion normalized() const;
    void normalize();

    static Quaternion slerp(const Quaternion& a, const Quaternion& b, double t);
    static double angularDistance(const Quaternion& a, const Quaternion& b);
    void rotateVector2D(double vx, double vy, double& out_vx, double & out_vy) const;
    static Quaternion weightedAverage(const Quaternion* qs, const double* weights, int count);
};
