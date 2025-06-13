#include "attitude_estimation/filters/kalman1d.hpp"
#include <Arduino.h>

KalmanFilter1D::KalmanFilter1D() : dt{0.004}, roll{0.0}, pitch{0.0}, yaw{0.0}
{
    P_roll = 16.0f;  // 4 * 4
    P_pitch = 16.0f; // 4 * 4
    R = 9.0f;        // 3 * 3
    Q = dt * dt * 4 * 4;
}

void KalmanFilter1D::update(const float &gx, const float &gy, const float &gz, const float &ax, const float &ay, const float &az)
{
    // Predict
    roll += gx * dt;
    P_roll += Q;

    pitch += gy * dt;
    P_pitch += Q;

    // Measure
    float roll_acc = atan2(ay, az) * RAD_TO_DEG;
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    // Update roll
    float y_roll = roll_acc - roll; // residual
    float K_roll = P_roll / (P_roll + R);
    roll += K_roll * y_roll;
    P_roll = (1 - K_roll) * P_roll;

    // Update pitch
    float y_pitch = pitch_acc - pitch;
    float K_pitch = P_pitch / (P_pitch + R);
    pitch += K_pitch * y_pitch;
    P_pitch = (1 - K_pitch) * P_pitch;

    // Update yaw 
    yaw += gz * dt;
    
    // Normalize yaw to -180 to 180 degrees
    yaw = fmod(yaw, 360.0f);
    if (yaw > 180.0f)
        yaw -= 360.0f;
    else if (yaw < -180.0f)
        yaw += 360.0f;
}

void KalmanFilter1D::reset(const float &gx, const float &gy, const float &gz, const float &ax, const float &ay, const float &az)
{

    P_roll = 16.0f;  // 4 * 4
    P_pitch = 16.0f; // 4 * 4
    
}