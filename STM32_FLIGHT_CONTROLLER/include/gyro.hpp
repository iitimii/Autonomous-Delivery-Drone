#ifndef GYRO_HPP
#define GYRO_HPP
#include <Arduino.h>
namespace gyro
{
    extern float pitch;
    extern float roll;
    extern float yaw;

    extern int16_t pitch_rate;
    extern int16_t roll_rate;
    extern int16_t yaw_rate;
    
    extern float pitch_accelerometer;
    extern float roll_accelerometer;

    extern int16_t acc_x;
    extern int16_t acc_y;
    extern int16_t acc_z;
    extern int32_t acc_resultant;

    extern bool use_manual_calibration;

    extern int32_t roll_rate_cal;
    extern int32_t pitch_rate_cal;
    extern int32_t yaw_rate_cal;

    extern int32_t roll_accelerometer_cal;
    extern int32_t pitch_accelerometer_cal;

    extern float roll_rate_lpf;
    extern float pitch_rate_lpf;
    extern float yaw_rate_lpf;
    extern int16_t temperature;

    void setup();
    void read();
    void calibrate();
    void get_acc_angle();
    void filter();
    void calculate_attitude();
    void calibrate_manual();
}

#endif // GYRO_HPP
