#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP
#include <Arduino.h>

namespace pid
{
    struct PIDGains
    {
        float kp, ki, kd;
    };

    struct PIDOutput
    {
        int16_t pitch, roll, throttle, yaw;

        struct Angle
        {
            int16_t pitch, roll;
        } angle;
    };

    struct PIDGain
    {
        struct Rate
        {
            PIDGains pitch, roll, yaw;
        } rate;

        struct Angle
        {
            PIDGains pitch, roll;
        } angle;
    };

    extern PIDOutput output;
    extern PIDGain gain;

} // namespace pid

#endif // PID_CONTROLLER_HPP
