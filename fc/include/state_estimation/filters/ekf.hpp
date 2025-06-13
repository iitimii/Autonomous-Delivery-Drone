#ifndef QUADROTOR_EKF_HPP
#define QUADROTOR_EKF_HPP

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

class EKF
{
private:
    Zeros<6, 15> h_;
    Zeros<6, 6> r_;
    Zeros<12, 12> rw_;
    Zeros<15, 12> gs_;
    Zeros<6, 6> s_;
    Zeros<15, 15> p_;
    Zeros<15, 15> q_;
    Zeros<15, 6> k_;
    Zeros<15, 15> fs_;
    Zeros<15, 15> phi_;
    Zeros<6, 1> y_;
    Zeros<15, 1> x_;

    Matrix<3, 1> GRAV_NED_MPS2_ = {0.0f, 0.0f, 9.81f};

    Matrix<3, 3> t_b2ned;
    Zeros<3, 1> accel_bias_mps2_;
    Matrix<3, 1> gyro_bias_radps_;
    Matrix<3, 1> accel_norm_mps2_;
    Matrix<3, 1> mag_norm_mps2_;
    Matrix<4, 1> delta_quat_;
    Matrix<4, 1> quat_;

    Matrix<3, 1> ins_accel_mps2_;
    Matrix<3, 1> ins_gyro_radps_;
    Matrix<3, 1> ins_ypr_rad_;
    Matrix<3, 1> ins_ned_vel_mps_;
    Matrix<3, 1, double> ins_lla_rad_m_;

public:
    EKF();

    void init();
    void predict();
    void update();
    void run(const float &gx, const float &gy, const float &gz,
             const float &ax, const float &ay, const float &az,
             const float &mx, const float &my, const float &mz,
            const float &gps_enu_x, const float &gps_enu_y, const float &gps_enu_z,
            const float &baro_altitude
        );
};

#endif // QUADROTOR_EKF_HPP