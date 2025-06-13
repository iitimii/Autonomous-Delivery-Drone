#include "controllers/pid.hpp"

PIDController::PIDController(float kp, float ki, float kd)
    : kp{kp}, ki{ki}, kd{kd}, dt{0.004}, i_max{400.0}, integral{0.0}, prev_error{0.0}
{
}

float PIDController::calculate(float setpoint, float state)
{
    float error = setpoint - state;
    float proportional = kp * error;

    if (ki == 0.0f && kd == 0.0f)
    {
        return proportional;
    }

    integral += ki * (error + prev_error) * dt * 0.5f;
    integral = constrain(integral, -i_max, i_max);

    float derivative = kd * (error - prev_error) / dt;
    prev_error = error;
    return proportional + integral + derivative;
}

void PIDController::reset()
{
    integral = 0;
    prev_error = 0;
}

void PIDController::setPIDgains(float kp_n, float ki_n, float kd_n)
{
    this->kp = kp_n;
    this->ki = ki_n;
    this->kd = kd_n;
}
