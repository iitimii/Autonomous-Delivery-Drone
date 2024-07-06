#include <Arduino.h>
#include <math.h>

class KalmanFilter2D
{
private:
    float dt;
    float roll, pitch, yaw;
    float P[4]; // 2x2 covariance matrix
    float Q[4]; // 2x2 process noise covariance
    float R[4]; // 2x2 measurement noise covariance

    // Helper function for 2x2 matrix multiplication
    void matrixMultiply(const float A[4], const float B[4], float result[4])
    {
        result[0] = A[0] * B[0] + A[1] * B[2];
        result[1] = A[0] * B[1] + A[1] * B[3];
        result[2] = A[2] * B[0] + A[3] * B[2];
        result[3] = A[2] * B[1] + A[3] * B[3];
    }

    // Helper function for 2x2 matrix addition
    void matrixAdd(const float A[4], const float B[4], float result[4])
    {
        for (int i = 0; i < 4; i++)
        {
            result[i] = A[i] + B[i];
        }
    }

    // Helper function for 2x2 matrix subtraction
    void matrixSubtract(const float A[4], const float B[4], float result[4])
    {
        for (int i = 0; i < 4; i++)
        {
            result[i] = A[i] - B[i];
        }
    }

    // Helper function for 2x2 matrix inversion
    void matrixInverse(const float A[4], float result[4])
    {
        float det = A[0] * A[3] - A[1] * A[2];
        if (fabs(det) < 1e-6f)
        {
            // Matrix is not invertible, use identity matrix
            result[0] = 1.0f;
            result[1] = 0.0f;
            result[2] = 0.0f;
            result[3] = 1.0f;
        }
        else
        {
            float invDet = 1.0f / det;
            result[0] = A[3] * invDet;
            result[1] = -A[1] * invDet;
            result[2] = -A[2] * invDet;
            result[3] = A[0] * invDet;
        }
    }

public:
    KalmanFilter2D() : dt(0.004f), roll(0.0f), pitch(0.0f)
    {
        // Initialize P matrix
        P[0] = 1.0f;
        P[1] = 0.0f;
        P[2] = 0.0f;
        P[3] = 1.0f;

        // Initialize Q matrix (process noise)
        float q = 0.001f; // Adjust this value based on your system
        Q[0] = q;
        Q[1] = 0.0f;
        Q[2] = 0.0f;
        Q[3] = q;

        // Initialize R matrix (measurement noise)
        float r = 0.03f; // Adjust this value based on your sensors
        R[0] = r;
        R[1] = 0.0f;
        R[2] = 0.0f;
        R[3] = r;
    }

    void update(const float &gx, const float &gy, const float &gz, const float &ax, const float &ay, const float &az)
    {
        // Predict step
        float sin_roll = sin(roll * DEG_TO_RAD);
        float cos_roll = cos(roll * DEG_TO_RAD);
        float cos_pitch = cos(pitch * DEG_TO_RAD);
        float tan_pitch = tan(pitch * DEG_TO_RAD);

        float roll_dot = gx + gy * sin_roll * tan_pitch + gz * cos_roll * tan_pitch;
        float pitch_dot = gy * cos_roll - gz * sin_roll;

        roll += roll_dot * dt;
        pitch += pitch_dot * dt;

        // Compute Jacobian A
        float A[4];
        A[0] = 1.0f + dt * (gy * cos_roll * tan_pitch - gz * sin_roll * tan_pitch);
        A[1] = dt * (gy * sin_roll / cos_pitch / cos_pitch + gz * cos_roll / cos_pitch / cos_pitch);
        A[2] = dt * (-gy * sin_roll - gz * cos_roll);
        A[3] = 1.0f;

        // Update P matrix
        float temp[4], temp2[4];
        matrixMultiply(A, P, temp);
        matrixMultiply(temp, A, temp2);
        matrixAdd(temp2, Q, P);

        // Measurement step
        float roll_m = atan2(ay, az) * RAD_TO_DEG;
        float pitch_m = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

        // Compute Kalman gain
        float S[4], K[4];
        matrixAdd(P, R, S);
        matrixInverse(S, temp);
        matrixMultiply(P, temp, K);

        // Update state
        float y[2] = {roll_m - roll, pitch_m - pitch};
        roll += K[0] * y[0] + K[1] * y[1];
        pitch += K[2] * y[0] + K[3] * y[1];

        // Update P matrix
        float I[4] = {1.0f, 0.0f, 0.0f, 1.0f};
        matrixMultiply(K, P, temp);
        matrixSubtract(I, temp, temp2);
        matrixMultiply(temp2, P, P);

        // Normalize angles
        roll = fmod(roll, 360.0f);
        if (roll > 180.0f)
            roll -= 360.0f;
        else if (roll < -180.0f)
            roll += 360.0f;

        pitch = fmod(pitch, 360.0f);
        if (pitch > 180.0f)
            pitch -= 360.0f;
        else if (pitch < -180.0f)
            pitch += 360.0f;

        // Update yaw (simple integration, no Kalman filter applied)
        yaw += gz * dt;

        // Normalize yaw to -180 to 180 degrees
        yaw = fmod(yaw, 360.0f);
        if (yaw > 180.0f)
            yaw -= 360.0f;
        else if (yaw < -180.0f)
            yaw += 360.0f;
    }

    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
};