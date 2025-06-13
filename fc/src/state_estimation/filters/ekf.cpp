#include "state_estimation/filters/ekf.hpp"
#include <BasicLinearAlgebra.h>
using namespace BLA;

EKF::EKF() {}

void EKF::init()
{
    // Zero initialize everything (BLA does this already for Zeros<>, but being explicit is good)
    x_ = Zeros<15,1>();
    p_ = 0.1f * Identity<15>();
    q_ = 0.01f * Identity<15>();
    r_ = 0.05f * Identity<6>();
    gs_ = Zeros<15, 12>();
    fs_ = Identity<15>();
    phi_ = Identity<15>();

    // Initial attitude (flat, pointing forward)
    quat_ = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    delta_quat_ = {1.0f, 0.0f, 0.0f, 0.0f};

    accel_bias_mps2_ = Zeros<3,1>();
    gyro_bias_radps_ = Zeros<3,1>();

    ins_accel_mps2_ = Zeros<3,1>();
    ins_gyro_radps_ = Zeros<3,1>();
    ins_ypr_rad_ = Zeros<3,1>();
    ins_ned_vel_mps_ = Zeros<3,1>();
    ins_lla_rad_m_ = Zeros<3,1,double>();
}

void EKF::predict()
{
    float dt = 0.004f;

    // 1. Subtract gyro bias
    Matrix<3,1> omega = ins_gyro_radps_ - gyro_bias_radps_;

    // 2. Update orientation (Euler step)
    Matrix<4,1> q = quat_;
    Matrix<4,4> Omega = {
        {  0.0f, -omega(0), -omega(1), -omega(2)},
        {omega(0),   0.0f,  omega(2), -omega(1)},
        {omega(1), -omega(2),  0.0f,  omega(0)},
        {omega(2),  omega(1), -omega(0), 0.0f}
    };
    quat_ = q + 0.5f * Omega * q * dt;

    // Normalize quaternion
    float norm = sqrt(quat_(0)*quat_(0) + quat_(1)*quat_(1) + quat_(2)*quat_(2) + quat_(3)*quat_(3));
    quat_ = quat_ / norm;

    // 3. Compute DCM (Rotation matrix) from quaternion
    float qw = quat_(0), qx = quat_(1), qy = quat_(2), qz = quat_(3);
    t_b2ned = {
        {1 - 2*qy*qy - 2*qz*qz,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw},
        {2*qx*qy + 2*qz*qw,     1 - 2*qx*qx - 2*qz*qz,     2*qy*qz - 2*qx*qw},
        {2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy}
    };

    // 4. Subtract accel bias
    Matrix<3,1> accel = ins_accel_mps2_ - accel_bias_mps2_;

    // 5. Rotate acceleration to NED
    Matrix<3,1> accel_ned = t_b2ned * accel;

    // 6. Subtract gravity
    Matrix<3,1> accel_lin = accel_ned - GRAV_NED_MPS2_;

    // 7. State propagation (position and velocity)
    x_(0) += x_(3) * dt;  // pos_x += vel_x
    x_(1) += x_(4) * dt;
    x_(2) += x_(5) * dt;

    x_(3) += accel_lin(0) * dt;  // vel_x += accel
    x_(4) += accel_lin(1) * dt;
    x_(5) += accel_lin(2) * dt;

    // TODO: Add quaternion error propagation and bias states

    // 8. Covariance update
    // Here, phi_ is your F matrix (linearized), assumed Identity for now
    p_ = phi_ * p_ * Transpose(phi_) + q_;
}

void EKF::update()
{
    // Measurements: accelerometer and magnetometer direction
    // accel_norm_mps2_ and mag_norm_mps2_ are unit vectors in body frame

    // 1. Predicted gravity vector in body frame
    Matrix<3,1> g_ned = GRAV_NED_MPS2_ / 9.81f;  // Normalize
    Matrix<3,1> g_body = Transpose(t_b2ned) * g_ned;

    // 2. Predicted mag vector (assuming known field, e.g., [1, 0, 0] in NED)
    Matrix<3,1> mag_ned = {1.0f, 0.0f, 0.0f};
    Matrix<3,1> mag_body = Transpose(t_b2ned) * mag_ned;

    // 3. Form measurement prediction
    h_.Submatrix<3,1>(0,0) = g_body;
    h_.Submatrix<3,1>(3,0) = mag_body;

    // 4. Innovation
    y_.Submatrix<3,1>(0,0) = accel_norm_mps2_ - g_body;
    y_.Submatrix<3,1>(3,0) = mag_norm_mps2_ - mag_body;

    // 5. Simple H approximation: identity where gravity/mag affects orientation
    // Here we pretend only 6x6 submatrix matters
    s_ = h_ * Transpose(h_) + r_;
    k_ = p_ * Transpose(h_) * Invert(s_);

    // 6. Update state
    x_ = x_ + k_ * y_;

    // 7. Update covariance
    Matrix<15,15> I = Identity<15>();
    p_ = (I - k_ * h_) * p_;
}
