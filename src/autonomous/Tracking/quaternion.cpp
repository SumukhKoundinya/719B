#include "main.h"

Quaternion::Quaternion()
    : w(1.0), x(0.0), y(0.0), z(0.0) {}

Quaternion::Quaternion(double w, double x, double y, double z)
    : w(w), x(x), y(y), z(z) {}

Quaternion Quaternion::fromHeading(double theta) {
    double half = theta * 0.5;
    return Quaternion(std::cos(half), 0.0, 0.0, std::sin(half));
}

Quaternion Quaternion::identity() {
    return Quaternion(1.0, 0.0, 0.0, 0.0);
}

double Quaternion::toHeading() const {
    return 2.0 * std::atan2(z, w);
}

Quaternion Quaternion::operator*(const Quaternion& r) const {
    return Quaternion(
        w*r.w - x*r.x - y*r.y - z*r.z,
        w*r.x + x*r.w + y*r.z - z*r.y,
        w*r.y - x*r.z + y*r.w + z*r.x,
        w*r.z + x*r.y - y*r.x + z*r.w
    );
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

double Quaternion::normSq() const {
    return w*w + x*x + y*y + z*z;
}

double Quaternion::norm() const {
    return std::sqrt(normSq());
}

Quaternion Quaternion::normalized() const {
    double n = norm();
    if (n < 1e-10) return identity();
    double inv = 1.0 / n;
    return Quaternion(w*inv, x*inv, y*inv, z*inv);
}

void Quaternion::normalize() {
    double n = norm();
    if (n < 1e-10) { *this = identity(); return; }
    double inv = 1.0 / n;
    w *= inv; x *= inv; y *= inv; z *= inv;
}

Quaternion Quaternion::slerp(const Quaternion& a, const Quaternion& b, double t) {
    double dot = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;

    Quaternion b2 = b;
    if (dot < 0.0) {
        b2.w = -b.w; b2.x = -b.x; b2.y = -b.y; b2.z = -b.z;
        dot = -dot;
    }

    if (dot > 0.9995) {
        return Quaternion(
            a.w + t*(b2.w - a.w),
            a.x + t*(b2.x - a.x),
            a.y + t*(b2.y - a.y),
            a.z + t*(b2.z - a.z)
        ).normalized();
    }

    double angle  = std::acos(dot);
    double sinInv = 1.0 / std::sin(angle);
    double s1     = std::sin((1.0 - t) * angle) * sinInv;
    double s2     = std::sin(t * angle) * sinInv;

    return Quaternion(
        s1*a.w + s2*b2.w,
        s1*a.x + s2*b2.x,
        s1*a.y + s2*b2.y,
        s1*a.z + s2*b2.z
    );
}

double Quaternion::angularDistance(const Quaternion& a, const Quaternion& b) {
    Quaternion r = a.conjugate() * b;
    double absW = std::fabs(r.w);
    if (absW > 1.0) absW = 1.0;
    return 2.0 * std::acos(absW);
}

void Quaternion::rotateVector2D(double vx, double vy, double& out_vx, double& out_vy) const {
    Quaternion p(0.0, vx, vy, 0.0);
    Quaternion result = (*this) * p * this->conjugate();
    out_vx = result.x;
    out_vy = result.y;
}

Quaternion Quaternion::weightedAverage(const Quaternion* qs, const double* weights, int count) {
    double sw = 0.0, sx = 0.0, sy = 0.0, sz = 0.0;
    const Quaternion& ref = qs[0];

    for (int i = 0; i < count; ++i) {
        double dot = ref.w*qs[i].w + ref.x*qs[i].x
                   + ref.y*qs[i].y + ref.z*qs[i].z;
        double sign = (dot < 0.0) ? -1.0 : 1.0;
        sw += weights[i] * sign * qs[i].w;
        sx += weights[i] * sign * qs[i].x;
        sy += weights[i] * sign * qs[i].y;
        sz += weights[i] * sign * qs[i].z;
    }

    return Quaternion(sw, sx, sy, sz).normalized();
}


