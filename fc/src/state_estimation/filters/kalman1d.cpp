#include "state_estimation/filters/kalman1d.hpp"
#include <Arduino.h>
// TODO use quaternions to represent orientation

KalmanFilter1D::KalmanFilter1D() : dt{0.004}, roll{0.0}, pitch{0.0}, yaw{0.0}
{
    P_roll = 16.0f;  // 4 * 4
    P_pitch = 16.0f; // 4 * 4
    P_yaw = 16.0f;   // 4 * 4
    R = 9.0f;        // 3 * 3
    R_yaw = 20.0f;   // 3 * 3
    Q = 4.0f;        // dt * dt * 4 * 4;

    // Initialize constants
    mag_declination = -1.25f; // Magnetic declination for Nigeria in degrees
    mag_offset = -76.0f;       // Offset between drone's x-axis and magnetometer x-axis in degrees
}

void KalmanFilter1D::update(const float &gx, const float &gy, const float &gz, const float &ax, const float &ay, const float &az, const float &mx, const float &my, const float &mz)
{
    // Predict
    roll += gx * dt;
    P_roll += Q;
    pitch += gy * dt;
    P_pitch += Q;
    yaw += gz * dt;
    P_yaw += Q;

    // Measure roll and pitch
    float roll_acc = atan2(ay, az) * RAD_TO_DEG;
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    // Update roll and pitch
    float y_roll = roll_acc - roll; // residual
    float K_roll = P_roll / (P_roll + R);
    roll += K_roll * y_roll;
    P_roll = (1 - K_roll) * P_roll;

    float y_pitch = pitch_acc - pitch;
    float K_pitch = P_pitch / (P_pitch + R);
    pitch += K_pitch * y_pitch;
    P_pitch = (1 - K_pitch) * P_pitch;

    // Measure yaw
    float roll_rad = roll * DEG_TO_RAD;
    float pitch_rad = pitch * DEG_TO_RAD;
    float cos_roll = cos(roll_rad);
    float sin_roll = sin(roll_rad);
    float cos_pitch = cos(pitch_rad);
    float sin_pitch = sin(pitch_rad);

    float mx_h = mx * cos_pitch + mz * sin_pitch;
    float my_h = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;
    float mz_h = -mx * cos_roll * sin_pitch + my * sin_roll + mz * cos_roll * cos_pitch;

    float yaw_mag = atan2(my_h, mx_h) * RAD_TO_DEG + mag_offset;

    // Update yaw
    float y_yaw = yaw_mag - yaw;

    // Handle angle wrap-around for residual calculation
    if (y_yaw > 180.0f)
    {
        y_yaw -= 360.0f;
    }
    else if (y_yaw < -180.0f)
    {
        y_yaw += 360.0f;
    }

    float K_yaw = P_yaw / (P_yaw + R_yaw);
    yaw += K_yaw * y_yaw;
    P_yaw = (1 - K_yaw) * P_yaw;

    // Normalize yaw to -180 to 180 degrees
    yaw = fmod(yaw, 360.0f);
    if (yaw > 180.0f)
        yaw -= 360.0f;
    else if (yaw < -180.0f)
        yaw += 360.0f;
}

void KalmanFilter1D::init(const float &gx, const float &gy, const float &gz, const float &ax, const float &ay, const float &az, const float &mx, const float &my, const float &mz)
{
    float roll_acc = atan2(ay, az) * RAD_TO_DEG;
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    float roll_rad = roll_acc * DEG_TO_RAD;
    float pitch_rad = pitch_acc * DEG_TO_RAD;
    float mx_comp = mx * cos(pitch_rad) + my * sin(roll_rad) * sin(pitch_rad) + mz * cos(roll_rad) * sin(pitch_rad);
    float my_comp = my * cos(roll_rad) - mz * sin(roll_rad);
    float yaw_mag = atan2(my_comp, mx_comp) * RAD_TO_DEG;
    yaw_mag = atan2(my, mx) * RAD_TO_DEG;
    yaw_mag += mag_declination;
    yaw_mag += mag_offset;
    yaw_mag = fmod(yaw_mag, 360.0f);
    if (yaw_mag > 180.0f)
        yaw_mag -= 360.0f;
    else if (yaw_mag < -180.0f)
        yaw_mag += 360.0f;

    setRoll(roll_acc);
    setPitch(pitch_acc);
    setYaw(yaw_mag);
}

void KalmanFilter1D::reset(const float &gx, const float &gy, const float &gz, const float &ax, const float &ay, const float &az, const float &mx, const float &my, const float &mz)
{
}