#include "attitude_estimation/attitude_estimation.hpp"
#include <math.h>

AttitudeEstimator::AttitudeEstimator() {}

void AttitudeEstimator::setup()
{
    imu.setup();
    imu.calibrate();
    // filter.begin(250.0);
}

void AttitudeEstimator::update()
{
    imu.read();
    gx = imu.getGx(); // dps
    gy = imu.getGy();
    gz = imu.getGz();
    ax = imu.getAx();
    ay = imu.getAy();
    az = imu.getAz();
    temperature = imu.getTemperature();
    acc_resultant = sqrt((ax * ax) + (ay * ay) + (az * az));

    filter.update(gx, gy, gx, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();
}

void AttitudeEstimator::reset()
{
    imu.read();
    gx = imu.getGx(); // dps
    gy = imu.getGy();
    gz = imu.getGz();
    ax = imu.getAx();
    ay = imu.getAy();
    az = imu.getAz();
    
    filter.reset(gx, gy, gx, ax, ay, az);
}