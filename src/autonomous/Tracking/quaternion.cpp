#include "main.h"

Quaternion::Quaternion() : w(1), x(0), y(0), z(0) {}

Quaternion::Quaternion(double w, double x, double y, double z)
    : w(w), x(x), y(y), z(z) {}

void Quaternion::normalize() {
    double mag = std::sqrt(w*w + x*x + y*y + z*z);
    if (mag == 0) return;
    w /= mag;
    x /= mag;
    y /= mag;
    z /= mag;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::operator*(const Quaternion& o) const {
    return Quaternion(
        w*o.w - x*o.x - y*o.y - z*o.z,
        w*o.x + x*o.w + y*o.z - z*o.y,
        w*o.y - x*o.z + y*o.w + z*o.x,
        w*o.z + x*o.y - y*o.x + z*o.w
    );
}

Quaternion Quaternion::fromYaw(double radians) {
    return Quaternion(
        std::cos(radians / 2.0),
        0,
        0,
        std::sin(radians / 2.0)
    );
}
