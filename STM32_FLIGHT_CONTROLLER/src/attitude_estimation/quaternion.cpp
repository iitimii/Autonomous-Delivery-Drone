#include "attitude_estimation/quaternion.hpp"
#include <math.h>

Quaternion::Quaternion(float a, float b, float c, float d) 
    : q1(a), q2(b), q3(c), q4(d) {}

Quaternion Quaternion::operator*(const Quaternion& R) const {
    return {
        q1 * R.q1 - q2 * R.q2 - q3 * R.q3 - q4 * R.q4,
        q1 * R.q2 + q2 * R.q1 + q3 * R.q4 - q4 * R.q3,
        q1 * R.q3 - q2 * R.q4 + q3 * R.q1 + q4 * R.q2,
        q1 * R.q4 + q2 * R.q3 - q3 * R.q2 + q4 * R.q1
    };
}

Quaternion& Quaternion::operator*=(float scalar) {
    q1 *= scalar; q2 *= scalar; q3 *= scalar; q4 *= scalar;
    return *this;
}

Quaternion& Quaternion::operator+=(const Quaternion& R) {
    q1 += R.q1; q2 += R.q2; q3 += R.q3; q4 += R.q4;
    return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& R) {
    q1 -= R.q1; q2 -= R.q2; q3 -= R.q3; q4 -= R.q4;
    return *this;
}

Quaternion Quaternion::conjugate() const {
    return {q1, -q2, -q3, -q4};
}

float Quaternion::norm() const {
    return sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
}

void Quaternion::normalize() {
    float n = norm();
    *this *= (1.0f / n);
}

