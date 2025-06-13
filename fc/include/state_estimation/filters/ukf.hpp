#ifndef UKF_HPP
#define UKF_HPP
#include <Arduino.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

class UKF
{
private:
    static constexpr int n = 15;  // State dimension: [pos(3), vel(3), quat(4), omega(3), gyro_bias(3), accel_bias(3)]
    static constexpr int m = 10;  // Measurement dimension: [gps_pos(3), magnetometer(3), accelerometer(3), barometer(1)]
    static constexpr int L = 2*n + 1; // Number of sigma points
    
    // UKF parameters
    static constexpr float alpha = 1e-3f;
    static constexpr float beta = 2.0f;
    static constexpr float kappa = 0.0f;
    static constexpr float lambda = alpha * alpha * (n + kappa) - n;
    
    // Time step
    float dt = 1.0f / 250.0f; // 250Hz
    
    // State vector and covariance
    Matrix<n> x;      // State vector
    Matrix<n, n> P;   // State covariance
    
    // Sigma points
    Matrix<n, L> X;   // Sigma points
    Matrix<n, L> X_pred; // Predicted sigma points
    Matrix<m, L> Y;   // Measurement sigma points
    
    // Weights
    float Wm[L];      // Weights for mean
    float Wc[L];      // Weights for covariance
    
    // Process and measurement noise
    Matrix<n, n> Q;   // Process noise covariance
    Matrix<m, m> R;   // Measurement noise covariance
    
    // Working matrices
    Matrix<n> x_pred;
    Matrix<n, n> P_pred;
    Matrix<m> y_pred;
    Matrix<m, m> S;
    Matrix<n, m> K;
    Matrix<m> innovation;
    
    // Sensor data storage
    float gyro[3], accel[3], mag[3];
    float gps_pos[3], baro_alt;
    bool gps_valid, mag_valid, baro_valid;
    
    // Helper functions
    void computeSigmaPoints();
    void predictSigmaPoints();
    void predictMean();
    void predictCovariance();
    void predictMeasurement();
    void updateState();
    
    // Utility functions
    void normalizeQuaternion(Matrix<n>& state);
    Matrix<3, 3> quaternionToRotMatrix(const float q[4]);
    void rotateVector(const Matrix<3, 3>& R, const float in[3], float out[3]);
    float constrainAngle(float angle);
    
public:
    UKF();
    void init();
    void predict();
    void update();
    void run(const float &gx, const float &gy, const float &gz,
             const float &ax, const float &ay, const float &az,
             const float &mx, const float &my, const float &mz,
             const float &gps_enu_x, const float &gps_enu_y, const float &gps_enu_z,
             const float &baro_relative_altitude);
             
    // Getters for state estimates
    void getPosition(float pos[3]) const;
    void getVelocity(float vel[3]) const;
    void getQuaternion(float quat[4]) const;
    void getAngularVelocity(float omega[3]) const;
    void getGyroBias(float bias[3]) const;
    void getAccelBias(float bias[3]) const;
    void getEulerAngles(float &roll, float &pitch, float &yaw) const;
};

#endif // UKF_HPP