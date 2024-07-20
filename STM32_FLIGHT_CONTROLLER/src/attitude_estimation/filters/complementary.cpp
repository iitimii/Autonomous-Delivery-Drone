#include "attitude_estimation/filters/complementary.hpp"
#include <Arduino.h>

ComplementaryFilter::ComplementaryFilter() : _alpha{0.005}, dt{0.004}, roll{0.0}, pitch{0.0}, yaw{0.0}, roll_acc{0.0}, pitch_acc{0.0}
{
}

void ComplementaryFilter::update(const float &gx, const float &gy, const float &gz,
                                 const float &ax, const float &ay, const float &az)
{
    roll += gx * dt;
    pitch += gy * dt;
    yaw += gz * dt;

    const float acc_resultant = sqrt(ax * ax + ay * ay + az * az);

    if (acc_resultant > 0.01f) // Avoid division by near-zero
    {
        roll_acc = atan2(ay, az) * RAD_TO_DEG;
        pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

        roll = _alpha * roll + (1 - _alpha) * roll_acc;
        pitch = _alpha * pitch + (1 - _alpha) * pitch_acc;
    }
    // If acc_resultant is too small, skip accelerometer update

    // Normalize yaw to -180 to 180 degrees
    yaw = fmod(yaw, 360.0f);
    if (yaw > 180.0f)
        yaw -= 360.0f;
    else if (yaw < -180.0f)
        yaw += 360.0f;
}

void ComplementaryFilter::reset(const float &gx, const float &gy, const float &gz,
                                 const float &ax, const float &ay, const float &az)
{
    float roll_acc = atan2(ay, az) * RAD_TO_DEG;
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    roll = roll_acc;
    pitch = pitch_acc;
    yaw = 0;
}
