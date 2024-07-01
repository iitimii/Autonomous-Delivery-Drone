#ifndef GYRO_HPP
#define GYRO_HPP

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
    extern int32_t roll_rate_cal;
    extern int32_t pitch_rate_cal;
    extern int32_t yaw_rate_cal;
    extern double roll_rate_lpf;
    extern double pitch_rate_lpf;
    extern double yaw_rate_lpf;
    extern int16_t temperature;

    void setup();
    void read();
    void calibrate();
    void get_acc_angle();
    void filter();
    void calculate_attitude();
}

#endif // GYRO_HPP
