#include "controllers/mrac.hpp"

PIDController::PIDController(float kp, float ki, float kd)
    : kp{kp}, ki{ki}, kd{kd}, dt{0.004}, i_max{200}, integral{0}, prev_error{0}
{
}

float PIDController::calculate(float setpoint, float state)
{
    float error = setpoint - state;
    float proportional = kp * error;

    integral += ki * error * dt;
    integral = constrain(integral, -i_max, i_max);

    float derivative = kd * (error - prev_error) / dt;
    prev_error = error;
    return proportional + integral + derivative;
}

float PIDController::calculate(float setpoint, float state, float state_deriv)
{
    float error = setpoint - state;
    float proportional = kp * error;

    integral += ki * error * dt;
    integral = constrain(integral, -i_max, i_max);

    float derivative = kd * state_deriv;
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
