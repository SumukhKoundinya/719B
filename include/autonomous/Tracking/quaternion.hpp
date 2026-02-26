#pragma once
#include "main.h"

struct Quaternion {
    double w, x, y, z;

    Quaternion();
    Quaternion(double w, double x, double y, double z);

    void normalize();
    Quaternion conjugate() const;
    Quaternion operator*(const Quaternion& other) const;

    static Quaternion fromYaw(double radians);
};
