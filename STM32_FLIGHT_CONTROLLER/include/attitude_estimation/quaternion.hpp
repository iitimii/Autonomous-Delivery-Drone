#ifndef QUATERNION_HPP
#define QUATERNION_HPP


class Quaternion {
public:
    float q1, q2, q3, q4;

    Quaternion(float a = 1.0f, float b = 0.0f, float c = 0.0f, float d = 0.0f);

    Quaternion operator*(const Quaternion& R) const;
    Quaternion& operator*=(float scalar);
    Quaternion& operator+=(const Quaternion& R);
    Quaternion& operator-=(const Quaternion& R);

    Quaternion conjugate() const;
    float norm() const;
    void normalize();
    void print() const;
};

#endif // QUATERNION_HPP