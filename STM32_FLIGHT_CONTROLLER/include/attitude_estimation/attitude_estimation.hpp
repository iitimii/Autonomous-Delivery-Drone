#ifndef ATTITUDE_ESTIMATION_HPP
#define ATTITUDE_ESTIMATION_HPP
#include <cstdint>
#include "sensors/imu.hpp"
#include "filters/madgwick.hpp"
#include "filters/complementary.hpp"
#include "filters/kalman1d.hpp"
#include "filters/kalman2d.hpp"

class AttitudeEstimator
{
public:
    float gx, gy, gz;
    float ax, ay, az;
    float roll, pitch, yaw;
    float temperature, acc_resultant;

    IMU imu;
    KalmanFilter1D filter;
    // ComplementaryFilter filter;


    AttitudeEstimator();
    void setup();
    void update();
    void reset();
};

extern AttitudeEstimator attitude_estimator;

#endif // ATTITUDE_ESTIMATION_HPP