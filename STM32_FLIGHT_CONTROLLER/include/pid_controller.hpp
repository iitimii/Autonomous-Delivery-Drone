#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP
#include <Arduino.h>

namespace pid
{
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
        float channel_setpoint(uint16_t& channel_value);
    };

    struct PIDGains
    {
        float kp, ki, kd;
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

    extern PIDGain gain;

    void calculate();
    void setup();
    void reset();

} // namespace pid

#endif // PID_CONTROLLER_HPP
