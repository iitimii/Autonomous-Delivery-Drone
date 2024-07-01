#include "pid_controller.hpp"

namespace pid
{

PIDOutput output;
PIDGain gain;

}

class PIDController
{
private:
    float kp, ki, kd;
    float integral, prev_error;
    const float i_max;
    float dt;
    
public:

    PIDController(float kp, float ki, float kd, float dt = 0.004, float i_max = 400)
    : kp(kp), ki(ki), kd(kd), i_max(i_max), integral(0), prev_error(0), dt(dt)
    {

    }

    float calculate(int setpoint, float input)
    {
        float error = setpoint - input;
        float proportional = kp * error;

        if (ki == 0.0 && kd == 0.0) return proportional;
    
        integral += ki * ((error + prev_error)/2) * dt;
        // i += ki * error * dt;
        // Anti-windup: Limit integral term
        if (integral > i_max)
            integral = i_max;
        else if (integral < -i_max)
            integral = -i_max;
        float derivative = kd * (error - prev_error) / dt;
        prev_error = error;
        return proportional + integral + derivative;
    }

    void reset()
    {
        integral = 0;
        prev_error = 0;
    }

    void setPIDgains(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    int channel_setpoint(int channel_value)
    {
        int setpoint{0};
        if (channel_value > 1508)
            setpoint = channel_value - 1508;
        else if (channel_value < 1492)
            setpoint = channel_value - 1492;
        return setpoint;
    }
};


int roll_vel_setpoint, pitch_vel_setpoint, yaw_vel_setpoint;
int pid_roll_vel_output, pid_pitch_vel_output, pid_yaw_vel_output;

float kp_roll_vel = 3.0, ki_roll_vel = 0.05, kd_roll_vel = 0.1;
float kp_pitch_vel = kp_roll_vel, ki_pitch_vel = ki_roll_vel, kd_pitch_vel = kd_roll_vel;
float kp_yaw_vel = 3.0, ki_yaw_vel = 0.05, kd_yaw_vel = 1.0;

int roll_ang_setpoint, pitch_ang_setpoint;
int pid_roll_ang_output, pid_pitch_ang_output;
float kp_roll_ang =2.0, ki_roll_ang = 0.0, kd_roll_ang = 0.0;
float kp_pitch_ang = kp_roll_ang, ki_pitch_ang = ki_roll_ang, kd_pitch_ang = kd_roll_ang;


PIDController PID_roll_vel(kp_roll_vel, ki_roll_vel, kd_roll_vel);
PIDController PID_pitch_vel(kp_pitch_vel, ki_pitch_vel, kd_pitch_vel);
PIDController PID_yaw_vel(kp_yaw_vel, ki_yaw_vel, kd_yaw_vel);

PIDController PID_roll_ang(kp_roll_ang, ki_roll_ang, kd_roll_ang);
PIDController PID_pitch_ang(kp_pitch_ang, ki_pitch_ang, kd_pitch_ang);