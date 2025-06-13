#include "state_estimation/filters/ukf.hpp"
#include <math.h>

UKF::UKF() : gps_valid(false), mag_valid(false), baro_valid(false)
{
    // Initialize weights
    Wm[0] = lambda / (n + lambda);
    Wc[0] = lambda / (n + lambda) + (1 - alpha*alpha + beta);
    
    for(int i = 1; i < L; i++) {
        Wm[i] = 1.0f / (2.0f * (n + lambda));
        Wc[i] = 1.0f / (2.0f * (n + lambda));
    }
}

void UKF::init()
{
    // Initialize state vector
    x.Fill(0.0f);
    x(6) = 1.0f; // Initialize quaternion w component
    
    // Initialize state covariance with reasonable uncertainties
    P.Fill(0.0f);
    // Position uncertainty (m²)
    P(0,0) = P(1,1) = P(2,2) = 10.0f;
    // Velocity uncertainty (m²/s²)
    P(3,3) = P(4,4) = P(5,5) = 1.0f;
    // Quaternion uncertainty
    P(6,6) = P(7,7) = P(8,8) = P(9,9) = 0.1f;
    // Angular velocity uncertainty (rad²/s²)
    P(10,10) = P(11,11) = P(12,12) = 0.1f;
    // Gyro bias uncertainty (rad²/s²)
    P(13,13) = P(14,14) = P(15,15) = 0.01f;
    // Accel bias uncertainty (m²/s⁴)
    P(16,16) = P(17,17) = P(18,18) = 0.1f;
    
    // Process noise covariance
    Q.Fill(0.0f);
    // Position process noise (driven by velocity)
    Q(0,0) = Q(1,1) = Q(2,2) = 0.01f;
    // Velocity process noise
    Q(3,3) = Q(4,4) = Q(5,5) = 0.1f;
    // Quaternion process noise
    Q(6,6) = Q(7,7) = Q(8,8) = Q(9,9) = 0.001f;
    // Angular velocity process noise
    Q(10,10) = Q(11,11) = Q(12,12) = 0.01f;
    // Gyro bias process noise (random walk)
    Q(13,13) = Q(14,14) = Q(15,15) = 1e-6f;
    // Accel bias process noise (random walk)
    Q(16,16) = Q(17,17) = Q(18,18) = 1e-5f;
    
    // Measurement noise covariance
    R.Fill(0.0f);
    // GPS position noise (m²)
    R(0,0) = R(1,1) = R(2,2) = 4.0f;
    // Magnetometer noise (μT²)
    R(3,3) = R(4,4) = R(5,5) = 100.0f;
    // Accelerometer noise (m²/s⁴)
    R(6,6) = R(7,7) = R(8,8) = 0.01f;
    // Barometer noise (m²)
    R(9,9) = 0.25f;
}

void UKF::computeSigmaPoints()
{
    // Compute matrix square root of (n+λ)P using Cholesky decomposition
    Matrix<n, n> L_chol = P;
    L_chol *= (n + lambda);
    
    // Simple Cholesky decomposition for positive definite matrix
    for(int i = 0; i < n; i++) {
        for(int j = 0; j <= i; j++) {
            if(i == j) {
                float sum = 0.0f;
                for(int k = 0; k < j; k++) {
                    sum += L_chol(i,k) * L_chol(i,k);
                }
                L_chol(i,j) = sqrtf(L_chol(i,j) - sum);
            } else {
                float sum = 0.0f;
                for(int k = 0; k < j; k++) {
                    sum += L_chol(i,k) * L_chol(j,k);
                }
                L_chol(i,j) = (L_chol(i,j) - sum) / L_chol(j,j);
            }
        }
        // Zero upper triangle
        for(int j = i + 1; j < n; j++) {
            L_chol(i,j) = 0.0f;
        }
    }
    
    // Generate sigma points
    // X0 = x
    for(int i = 0; i < n; i++) {
        X(i, 0) = x(i);
    }
    
    // Xi = x + √((n+λ)P)_i for i = 1,...,n
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            X(j, i + 1) = x(j) + L_chol(j, i);
            X(j, i + 1 + n) = x(j) - L_chol(j, i);
        }
    }
    
    // Ensure quaternion sigma points are normalized
    for(int i = 0; i < L; i++) {
        float q_norm = sqrtf(X(6,i)*X(6,i) + X(7,i)*X(7,i) + X(8,i)*X(8,i) + X(9,i)*X(9,i));
        if(q_norm > 1e-6f) {
            X(6,i) /= q_norm;
            X(7,i) /= q_norm;
            X(8,i) /= q_norm;
            X(9,i) /= q_norm;
        }
    }
}

void UKF::predictSigmaPoints()
{
    for(int i = 0; i < L; i++) {
        // Extract state components
        float pos[3] = {X(0,i), X(1,i), X(2,i)};
        float vel[3] = {X(3,i), X(4,i), X(5,i)};
        float quat[4] = {X(6,i), X(7,i), X(8,i), X(9,i)};
        float omega[3] = {X(10,i), X(11,i), X(12,i)};
        float gyro_bias[3] = {X(13,i), X(14,i), X(15,i)};
        float accel_bias[3] = {X(16,i), X(17,i), X(18,i)};
        
        // Predict position: p_k+1 = p_k + v_k * dt
        X_pred(0,i) = pos[0] + vel[0] * dt;
        X_pred(1,i) = pos[1] + vel[1] * dt;
        X_pred(2,i) = pos[2] + vel[2] * dt;
        
        // Predict velocity: v_k+1 = v_k + (R * (a_imu - bias) + g) * dt
        Matrix<3,3> R = quaternionToRotMatrix(quat);
        float accel_corrected[3] = {
            gyro[0] - accel_bias[0],
            gyro[1] - accel_bias[1], 
            gyro[2] - accel_bias[2] - 9.81f  // Remove gravity in body frame
        };
        float accel_world[3];
        rotateVector(R, accel_corrected, accel_world);
        
        X_pred(3,i) = vel[0] + accel_world[0] * dt;
        X_pred(4,i) = vel[1] + accel_world[1] * dt;
        X_pred(5,i) = vel[2] + (accel_world[2] + 9.81f) * dt; // Add gravity back in world frame
        
        // Predict quaternion using angular velocity
        float omega_corrected[3] = {
            omega[0] - gyro_bias[0],
            omega[1] - gyro_bias[1],
            omega[2] - gyro_bias[2]
        };
        
        float omega_mag = sqrtf(omega_corrected[0]*omega_corrected[0] + 
                               omega_corrected[1]*omega_corrected[1] + 
                               omega_corrected[2]*omega_corrected[2]);
        
        if(omega_mag > 1e-6f) {
            float half_angle = 0.5f * omega_mag * dt;
            float s = sinf(half_angle) / omega_mag;
            float c = cosf(half_angle);
            
            // Quaternion multiplication for rotation
            float dq[4] = {c, s * omega_corrected[0], s * omega_corrected[1], s * omega_corrected[2]};
            
            X_pred(6,i) = quat[0]*dq[0] - quat[1]*dq[1] - quat[2]*dq[2] - quat[3]*dq[3];
            X_pred(7,i) = quat[0]*dq[1] + quat[1]*dq[0] + quat[2]*dq[3] - quat[3]*dq[2];
            X_pred(8,i) = quat[0]*dq[2] - quat[1]*dq[3] + quat[2]*dq[0] + quat[3]*dq[1];
            X_pred(9,i) = quat[0]*dq[3] + quat[1]*dq[2] - quat[2]*dq[1] + quat[3]*dq[0];
        } else {
            X_pred(6,i) = quat[0];
            X_pred(7,i) = quat[1];
            X_pred(8,i) = quat[2];
            X_pred(9,i) = quat[3];
        }
        
        // Normalize quaternion
        float q_norm = sqrtf(X_pred(6,i)*X_pred(6,i) + X_pred(7,i)*X_pred(7,i) + 
                            X_pred(8,i)*X_pred(8,i) + X_pred(9,i)*X_pred(9,i));
        if(q_norm > 1e-6f) {
            X_pred(6,i) /= q_norm;
            X_pred(7,i) /= q_norm;
            X_pred(8,i) /= q_norm;
            X_pred(9,i) /= q_norm;
        }
        
        // Angular velocity and biases remain constant (random walk model)
        X_pred(10,i) = omega[0];
        X_pred(11,i) = omega[1];
        X_pred(12,i) = omega[2];
        X_pred(13,i) = gyro_bias[0];
        X_pred(14,i) = gyro_bias[1];
        X_pred(15,i) = gyro_bias[2];
        X_pred(16,i) = accel_bias[0];
        X_pred(17,i) = accel_bias[1];
        X_pred(18,i) = accel_bias[2];
    }
}

void UKF::predictMean()
{
    x_pred.Fill(0.0f);
    
    // Compute weighted mean
    for(int i = 0; i < L; i++) {
        for(int j = 0; j < n; j++) {
            x_pred(j) += Wm[i] * X_pred(j, i);
        }
    }
    
    // Normalize quaternion in mean
    normalizeQuaternion(x_pred);
}

void UKF::predictCovariance()
{
    P_pred.Fill(0.0f);
    
    // Compute weighted covariance
    for(int i = 0; i < L; i++) {
        Matrix<n> diff;
        for(int j = 0; j < n; j++) {
            diff(j) = X_pred(j, i) - x_pred(j);
        }
        
        // Handle quaternion angle wrapping
        for(int j = 6; j < 10; j++) {
            diff(j) = constrainAngle(diff(j));
        }
        
        for(int j = 0; j < n; j++) {
            for(int k = 0; k < n; k++) {
                P_pred(j, k) += Wc[i] * diff(j) * diff(k);
            }
        }
    }
    
    // Add process noise
    P_pred += Q;
}

void UKF::predictMeasurement()
{
    // Generate measurement sigma points
    for(int i = 0; i < L; i++) {
        float pos[3] = {X_pred(0,i), X_pred(1,i), X_pred(2,i)};
        float quat[4] = {X_pred(6,i), X_pred(7,i), X_pred(8,i), X_pred(9,i)};
        float accel_bias[3] = {X_pred(16,i), X_pred(17,i), X_pred(18,i)};
        
        // GPS position measurement
        Y(0,i) = pos[0];
        Y(1,i) = pos[1];
        Y(2,i) = pos[2];
        
        // Magnetometer measurement (rotate expected magnetic field)
        Matrix<3,3> R = quaternionToRotMatrix(quat);
        float mag_earth[3] = {0.2f, 0.0f, 0.4f}; // Approximate magnetic field in ENU (example values)
        float mag_body[3];
        // Rotate from world to body frame (transpose of R)
        for(int j = 0; j < 3; j++) {
            mag_body[j] = 0.0f;
            for(int k = 0; k < 3; k++) {
                mag_body[j] += R(k,j) * mag_earth[k]; // Transpose
            }
        }
        Y(3,i) = mag_body[0];
        Y(4,i) = mag_body[1];
        Y(5,i) = mag_body[2];
        
        // Accelerometer measurement (gravity in body frame)
        float gravity_world[3] = {0.0f, 0.0f, -9.81f};
        float gravity_body[3];
        for(int j = 0; j < 3; j++) {
            gravity_body[j] = 0.0f;
            for(int k = 0; k < 3; k++) {
                gravity_body[j] += R(k,j) * gravity_world[k]; // Transpose
            }
        }
        Y(6,i) = gravity_body[0] + accel_bias[0];
        Y(7,i) = gravity_body[1] + accel_bias[1];
        Y(8,i) = gravity_body[2] + accel_bias[2];
        
        // Barometer measurement
        Y(9,i) = pos[2]; // Altitude
    }
    
    // Compute predicted measurement mean
    y_pred.Fill(0.0f);
    for(int i = 0; i < L; i++) {
        for(int j = 0; j < m; j++) {
            y_pred(j) += Wm[i] * Y(j, i);
        }
    }
    
    // Compute innovation covariance
    S.Fill(0.0f);
    for(int i = 0; i < L; i++) {
        Matrix<m> diff;
        for(int j = 0; j < m; j++) {
            diff(j) = Y(j, i) - y_pred(j);
        }
        
        for(int j = 0; j < m; j++) {
            for(int k = 0; k < m; k++) {
                S(j, k) += Wc[i] * diff(j) * diff(k);
            }
        }
    }
    S += R;
    
    // Compute cross-covariance
    Matrix<n, m> Pxy;
    Pxy.Fill(0.0f);
    for(int i = 0; i < L; i++) {
        Matrix<n> x_diff;
        Matrix<m> y_diff;
        
        for(int j = 0; j < n; j++) {
            x_diff(j) = X_pred(j, i) - x_pred(j);
        }
        for(int j = 0; j < m; j++) {
            y_diff(j) = Y(j, i) - y_pred(j);
        }
        
        // Handle quaternion angle wrapping
        for(int j = 6; j < 10; j++) {
            x_diff(j) = constrainAngle(x_diff(j));
        }
        
        for(int j = 0; j < n; j++) {
            for(int k = 0; k < m; k++) {
                Pxy(j, k) += Wc[i] * x_diff(j) * y_diff(k);
            }
        }
    }
    
    // Compute Kalman gain: K = Pxy * S^-1
    // For efficiency, we'll use a simplified inverse for diagonal-dominant matrices
    Matrix<m, m> S_inv;
    S_inv.Fill(0.0f);
    for(int i = 0; i < m; i++) {
        S_inv(i, i) = 1.0f / S(i, i);
    }
    
    K = Pxy * S_inv;
}

void UKF::updateState()
{
    // Compute innovation
    Matrix<m> z; // Actual measurements
    z(0) = gps_pos[0]; z(1) = gps_pos[1]; z(2) = gps_pos[2];
    z(3) = mag[0]; z(4) = mag[1]; z(5) = mag[2];
    z(6) = accel[0]; z(7) = accel[1]; z(8) = accel[2];
    z(9) = baro_alt;
    
    for(int i = 0; i < m; i++) {
        innovation(i) = z(i) - y_pred(i);
    }
    
    // Update state
    Matrix<n> K_innovation = K * innovation;
    x = x_pred + K_innovation;
    
    // Normalize quaternion
    normalizeQuaternion(x);
    
    // Update covariance
    Matrix<n, n> I;
    I.Fill(0.0f);
    for(int i = 0; i < n; i++) {
        I(i, i) = 1.0f;
    }
    
    P = (I - K * (~K)) * P_pred; // Simplified Joseph form
}

void UKF::predict()
{
    computeSigmaPoints();
    predictSigmaPoints();
    predictMean();
    predictCovariance();
}

void UKF::update()
{
    predictMeasurement();
    updateState();
}

void UKF::run(const float &gx, const float &gy, const float &gz,
              const float &ax, const float &ay, const float &az,
              const float &mx, const float &my, const float &mz,
              const float &gps_enu_x, const float &gps_enu_y, const float &gps_enu_z,
              const float &baro_relative_altitude)
{
    // Store sensor data
    gyro[0] = gx; gyro[1] = gy; gyro[2] = gz;
    accel[0] = ax; accel[1] = ay; accel[2] = az;
    mag[0] = mx; mag[1] = my; mag[2] = mz;
    gps_pos[0] = gps_enu_x; gps_pos[1] = gps_enu_y; gps_pos[2] = gps_enu_z;
    baro_alt = baro_relative_altitude;
    
    // Update sensor validity flags
    gps_valid = !isnan(gps_enu_x) && !isnan(gps_enu_y) && !isnan(gps_enu_z);
    mag_valid = !isnan(mx) && !isnan(my) && !isnan(mz);
    baro_valid = !isnan(baro_relative_altitude);
    
    // Run prediction step
    predict();
    
    // Run update step if measurements are available
    if(gps_valid || mag_valid || baro_valid) {
        update();
    }
}

// Utility functions
void UKF::normalizeQuaternion(Matrix<n>& state)
{
    float norm = sqrtf(state(6)*state(6) + state(7)*state(7) + state(8)*state(8) + state(9)*state(9));
    if(norm > 1e-6f) {
        state(6) /= norm;
        state(7) /= norm;
        state(8) /= norm;
        state(9) /= norm;
    }
}

Matrix<3, 3> UKF::quaternionToRotMatrix(const float q[4])
{
    Matrix<3, 3> R;
    float w = q[0], x = q[1], y = q[2], z = q[3];
    
    R(0,0) = 1 - 2*(y*y + z*z);
    R(0,1) = 2*(x*y - w*z);
    R(0,2) = 2*(x*z + w*y);
    R(1,0) = 2*(x*y + w*z);
    R(1,1) = 1 - 2*(x*x + z*z);
    R(1,2) = 2*(y*z - w*x);
    R(2,0) = 2*(x*z - w*y);
    R(2,1) = 2*(y*z + w*x);
    R(2,2) = 1 - 2*(x*x + y*y);
    
    return R;
}

void UKF::rotateVector(const Matrix<3, 3>& R, const float in[3], float out[3])
{
    out[0] = R(0,0)*in[0] + R(0,1)*in[1] + R(0,2)*in[2];
    out[1] = R(1,0)*in[0] + R(1,1)*in[1] + R(1,2)*in[2];
    out[2] = R(2,0)*in[0] + R(2,1)*in[1] + R(2,2)*in[2];
}

float UKF::constrainAngle(float angle)
{
    while(angle > M_PI) angle -= 2*M_PI;
    while(angle < -M_PI) angle += 2*M_PI;
    return angle;
}

// Getter functions
void UKF::getPosition(float pos[3]) const
{
    pos[0] = x(0); pos[1] = x(1); pos[2] = x(2);
}

void UKF::getVelocity(float vel[3]) const
{
    vel[0] = x(3); vel[1] = x(4); vel[2] = x(5);
}

void UKF::getQuaternion(float quat[4]) const
{
    quat[0] = x(6); quat[1] = x(7); quat[2] = x(8); quat[3] = x(9);
}

void UKF::getAngularVelocity(float omega[3]) const
{
    omega[0] = x(10); omega[1] = x(11); omega[2] = x(12);
}

void UKF::getGyroBias(float bias[3]) const
{
    bias[0] = x(13); bias[1] = x(14); bias[2] = x(15);
}

void UKF::getAccelBias(float bias[3]) const
{
    bias[0] = x(16); bias[1] = x(17); bias[2] = x(18);
}

void UKF::getEulerAngles(float &roll, float &pitch, float &yaw) const
{
    float w = x(6), x_ = x(7), y = x(8), z = x(9);
    
    // Roll (rotation about x-axis)
    roll = atan2f(2*(w*x_ + y*z), 1 - 2*(x_*x_ + y*y));
    
    // Pitch (rotation about y-axis)
    float sin_pitch = 2*(w*y - z*x_);
    sin_pitch = constrain(sin_pitch, -1.0f, 1.0f);
    pitch = asinf(sin_pitch);
    
    // Yaw (rotation about z-axis)
    yaw = atan2f(2*(w*z + x_*y), 1 - 2*(y*y + z*z));
}