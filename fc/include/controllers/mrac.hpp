#ifndef MRAC_CONTROLLER_HPP
#define MRAC_CONTROLLER_HPP
#include <Arduino.h>

class MRACController
{
private:
    float kp, ki, kd;
    float integral, prev_error;
    const float i_max;
    float dt;

public:
    MRACController(float kp, float ki, float kd);

    float calculate(float setpoint, float state);
    float calculate(float setpoint, float state, float state_deriv);
    void reset();
    void setMRACgains(float kp_n, float ki_n, float kd_n);
};

class MRACGain
{
public:
    float kp, ki, kd;
    MRACGain(float kp, float ki, float kd) : kp{kp}, ki{ki}, kd{kd} {};
};

#endif // MRAC_CONTROLLER_HPP
