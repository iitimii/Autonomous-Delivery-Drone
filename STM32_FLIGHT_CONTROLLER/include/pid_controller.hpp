#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP
#include <Arduino.h>

class PIDController
{
private:
    float kp, ki, kd;
    float integral, prev_error;
    const float i_max;
    float dt;
    
public:

    PIDController(float kp, float ki, float kd, float dt = 0.004, float i_max = 400);

    float calculate(int setpoint, float input);
    void reset();
    void setPIDgains(float kp, float ki, float kd);
    int channel_setpoint(int channel_value);
};

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
