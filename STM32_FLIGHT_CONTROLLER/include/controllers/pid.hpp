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
    PIDController(float kp, float ki, float kd);

    float calculate(float setpoint, float state);
    void reset();
    void setPIDgains(float kp_n, float ki_n, float kd_n);
};

class PIDGain
{
public:
    float kp, ki, kd;
    PIDGain(float kp, float ki, float kd) : kp{kp}, ki{ki}, kd{kd} {};
};

#endif // PID_CONTROLLER_HPP
