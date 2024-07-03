#include "pid_controller.hpp"
#include "outputs.hpp"
#include "gyro.hpp"
#include "receiver.hpp"

namespace pid
{
    PIDController::PIDController(float kp, float ki, float kd, float dt=0.004, float i_max=400)
        : kp(kp), ki(ki), kd(kd), dt(dt), i_max(i_max), integral(0), prev_error(0)
    {
    }

    float PIDController::calculate(int setpoint, float input)
    {
        float error = setpoint - input;
        float proportional = kp * error;

        if (ki != 0.0f || kd != 0.0f) {
            integral += ki * error * dt;
            integral = constrain(integral, -i_max, i_max);
            float derivative = kd * (error - prev_error) / dt;
            prev_error = error;
            return proportional + integral + derivative;
        }

        return proportional;
    }

    void PIDController::reset()
    {
        integral = 0;
        prev_error = 0;
    }

    void PIDController::setPIDgains(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    float PIDController::channel_setpoint(uint16_t& channel_value)
    {
        float setpoint{0};

        if (channel_value > 1508)
            setpoint = channel_value - 1508;
        else if (channel_value < 1492)
            setpoint = channel_value - 1492;

        return setpoint;
    }

    // Declare PID controllers
    PIDGain gain;

    PIDController roll_rate_controller(gain.rate.roll.kp, gain.rate.roll.ki, gain.rate.roll.kd);
    PIDController pitch_rate_controller(gain.rate.pitch.kp, gain.rate.pitch.ki, gain.rate.pitch.kd);
    PIDController yaw_rate_controller(gain.rate.yaw.kp, gain.rate.yaw.ki, gain.rate.yaw.kd);

    PIDController roll_angle_controller(gain.angle.roll.kp, gain.angle.roll.ki, gain.angle.roll.kd);
    PIDController pitch_angle_controller(gain.angle.pitch.kp, gain.angle.pitch.ki, gain.angle.pitch.kd);

    void setup()
    {
        gain.rate.roll = {5.0f, 0.05f, 0.0f};
        gain.rate.pitch = gain.rate.roll;
        gain.rate.yaw = {5.0f, 0.05f, 0.0f};

        gain.angle.roll = {2.0f, 0.0f, 0.0f};
        gain.angle.pitch = gain.angle.roll;

        roll_rate_controller.setPIDgains(gain.rate.roll.kp, gain.rate.roll.ki, gain.rate.roll.kd);
        pitch_rate_controller.setPIDgains(gain.rate.pitch.kp, gain.rate.pitch.ki, gain.rate.pitch.kd);
        yaw_rate_controller.setPIDgains(gain.rate.yaw.kp, gain.rate.yaw.ki, gain.rate.yaw.kd);

        roll_angle_controller.setPIDgains(gain.angle.roll.kp, gain.angle.roll.ki, gain.angle.roll.kd);
        pitch_angle_controller.setPIDgains(gain.angle.pitch.kp, gain.angle.pitch.ki, gain.angle.pitch.kd);
    }

    void calculate()
    {
        const float max_angle = 30.0f;
        float roll_ang_setpoint = roll_angle_controller.channel_setpoint(receiver::roll) * max_angle / 500.0f;
        float pitch_ang_setpoint = pitch_angle_controller.channel_setpoint(receiver::pitch) * max_angle / 500.0f;
        float yaw_vel_setpoint = yaw_rate_controller.channel_setpoint(receiver::yaw);

        outputs::angle.roll = roll_angle_controller.calculate(roll_ang_setpoint, gyro::roll);
        outputs::angle.pitch = pitch_angle_controller.calculate(pitch_ang_setpoint, gyro::pitch);

        outputs::roll = roll_rate_controller.calculate(outputs::angle.roll, gyro::roll_rate_lpf);
        outputs::pitch = pitch_rate_controller.calculate(outputs::angle.pitch, gyro::pitch_rate_lpf);
        outputs::yaw = yaw_rate_controller.calculate(yaw_vel_setpoint, gyro::yaw_rate_lpf);
        outputs::throttle = receiver::throttle;
    }

    void reset()
    {
        roll_rate_controller.reset();
        pitch_rate_controller.reset();
        yaw_rate_controller.reset();
        roll_angle_controller.reset();
        pitch_angle_controller.reset();
    }
}
