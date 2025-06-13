#include "actuators/outputs.hpp"
#include "communication/receiver.hpp"
#include <attitude_estimation/attitude_estimation.hpp>
#include "controllers/pid.hpp"

namespace control
{
    PIDGain roll_rate_gain {4.0, 0.1, 0.1};
    PIDGain pitch_rate_gain {4.0, 0.1, 0.1};
    PIDGain yaw_rate_gain {4.0, 0.1, 0.1};
    PIDGain roll_angle_gain {4.0, 0.1, 0.1};
    PIDGain pitch_angle_gain {4.0, 0.1, 0.1};

    PIDController roll_rate_controller{roll_rate_gain.kp, roll_rate_gain.ki, roll_rate_gain.kd};
    PIDController pitch_rate_controller{pitch_rate_gain.kp, pitch_rate_gain.ki, pitch_rate_gain.kd};
    PIDController yaw_rate_controller{yaw_rate_gain.kp, yaw_rate_gain.ki, yaw_rate_gain.kd};

    PIDController roll_angle_controller{roll_angle_gain.kp, roll_angle_gain.ki, roll_angle_gain.kd};
    PIDController pitch_angle_controller{pitch_angle_gain.kp, pitch_angle_gain.ki, pitch_angle_gain.kd};

    void calculate()
    {
        const float max_angle = 30.0f;
        const float max_rate = 50.0f;
        float deadzone = 10.0f;

        float roll_ang_setpoint = receiver::roll - 1500;
        float pitch_ang_setpoint = receiver::pitch - 1500;
        float yaw_rate_setpoint = receiver::yaw - 1500;

        if (roll_ang_setpoint < deadzone && roll_ang_setpoint > -deadzone)
            roll_ang_setpoint = 0.0f;
        if (pitch_ang_setpoint < deadzone && pitch_ang_setpoint > -deadzone)
            pitch_ang_setpoint = 0.0f;
        if (yaw_rate_setpoint < deadzone && yaw_rate_setpoint > -deadzone)
            yaw_rate_setpoint = 0.0f;

        roll_ang_setpoint = map(roll_ang_setpoint, -500, 500, -max_angle, max_angle);
        pitch_ang_setpoint = map(pitch_ang_setpoint, -500, 500, -max_angle, max_angle);
        yaw_rate_setpoint = map(yaw_rate_setpoint, -500, 500, -max_rate, max_rate);

        outputs::angle.roll = roll_angle_controller.calculate(roll_ang_setpoint, attitude_estimator.roll);
        outputs::angle.pitch = pitch_angle_controller.calculate(pitch_ang_setpoint, attitude_estimator.pitch);

        outputs::roll = roll_rate_controller.calculate(outputs::angle.roll, attitude_estimator.gx);
        outputs::pitch = pitch_rate_controller.calculate(outputs::angle.pitch, attitude_estimator.gy);
        outputs::yaw = yaw_rate_controller.calculate(yaw_rate_setpoint, attitude_estimator.gz);
        outputs::throttle = receiver::throttle;
    }

    void reset_controllers()
    {
        roll_rate_controller.reset();
        pitch_rate_controller.reset();
        yaw_rate_controller.reset();
        roll_angle_controller.reset();
        pitch_angle_controller.reset();
    }
} // namespace control