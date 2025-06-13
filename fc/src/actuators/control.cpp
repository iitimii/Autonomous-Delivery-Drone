#include "actuators/control.hpp"
#include "actuators/outputs.hpp"
#include "communication/receiver.hpp"
#include <state_estimation/state_estimation.hpp>
#include "controllers/pid.hpp"

namespace control
{
    // PIDGain roll_rate_gain {0.6, 3.5, 0.03};
    // PIDGain pitch_rate_gain {0.6, 3.5, 0.03};
    // PIDGain yaw_rate_gain {2.0, 12, 0.0};

    PIDGain roll_rate_gain {2.0, 0.0, 0.0};
    PIDGain pitch_rate_gain {2.0, 0.0, 0.0};
    PIDGain yaw_rate_gain {4.0, 0.01, 0.0};

    PIDGain roll_angle_gain {2.0, 0.0, 0.0};
    PIDGain pitch_angle_gain {2.0, 0.0, 0.0};

    // PIDGain roll_gain {2, 1.0, 0.0};
    // PIDGain pitch_gain {2, 1.0, 0.0};
    // PIDGain yaw_gain {2.0, 0.0, 0.0};

    PIDGain x_gain {0.0, 0, 0};
    PIDGain y_gain {0.0, 0, 0};
    PIDGain z_gain {0.0, 0, 0};

    PIDController roll_rate_controller{roll_rate_gain.kp, roll_rate_gain.ki, roll_rate_gain.kd};
    PIDController pitch_rate_controller{pitch_rate_gain.kp, pitch_rate_gain.ki, pitch_rate_gain.kd};
    PIDController yaw_rate_controller{yaw_rate_gain.kp, yaw_rate_gain.ki, yaw_rate_gain.kd};

    PIDController roll_angle_controller{roll_angle_gain.kp, roll_angle_gain.ki, roll_angle_gain.kd};
    PIDController pitch_angle_controller{pitch_angle_gain.kp, pitch_angle_gain.ki, pitch_angle_gain.kd};


    // PIDController roll_controller{roll_gain.kp, roll_gain.ki, roll_gain.kd};
    // PIDController pitch_controller{pitch_gain.kp, pitch_gain.ki, pitch_gain.kd};
    // PIDController yaw_controller{yaw_gain.kp, yaw_gain.ki, yaw_gain.kd};

    PIDController x_controller{x_gain.kp, x_gain.ki, x_gain.kd};
    PIDController y_controller{y_gain.kp, y_gain.ki, y_gain.kd};
    PIDController z_controller{z_gain.kp, z_gain.ki, z_gain.kd};

    // void attitude_control()
    // {
    //     const float max_angle = 30.0f;
    //     const float max_rate = 100.0f;
    //     float deadzone = 10.0f;

    //     float roll_setpoint = receiver::roll - 1500;
    //     float pitch_setpoint = receiver::pitch - 1500;
    //     float yaw_setpoint = receiver::yaw - 1500;

    //     if (roll_setpoint < deadzone && roll_setpoint > -deadzone)
    //         roll_setpoint = 0.0f;
    //     if (pitch_setpoint < deadzone && pitch_setpoint > -deadzone)
    //         pitch_setpoint = 0.0f;
    //     if (yaw_setpoint < deadzone && yaw_setpoint > -deadzone)
    //         yaw_setpoint = 0.0f;

    //     roll_setpoint = map(roll_setpoint, -500, 500, -max_angle, max_angle);
    //     pitch_setpoint = map(pitch_setpoint, -500, 500, -max_angle, max_angle);
    //     yaw_setpoint = map(yaw_setpoint, -500, 500, -max_rate, max_rate);


    //     outputs::roll = roll_controller.calculate(roll_setpoint, state_estimator.roll, state_estimator.gx);
    //     outputs::pitch = pitch_controller.calculate(pitch_setpoint, state_estimator.pitch, state_estimator.gy);
    //     outputs::yaw = yaw_controller.calculate(yaw_setpoint, state_estimator.gz);
    //     outputs::throttle = receiver::throttle;
    // }

    void attitude_control()
    {
        const float max_angle = 30.0f;
        const float max_rate = 100.0f;
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

        outputs::angle.roll = roll_angle_controller.calculate(roll_ang_setpoint, state_estimator.roll);
        outputs::angle.pitch = pitch_angle_controller.calculate(pitch_ang_setpoint, state_estimator.pitch);

        outputs::roll = roll_rate_controller.calculate(outputs::angle.roll, state_estimator.gx);
        outputs::pitch = pitch_rate_controller.calculate(outputs::angle.pitch, state_estimator.gy);
        outputs::yaw = yaw_rate_controller.calculate(yaw_rate_setpoint, state_estimator.gz);
        outputs::throttle = receiver::throttle;
    }

    void velocity_control()
    {
        // const float max_vel = 5.0f;
        // float x_dot_setpoint = receiver::x_dot - 1500;
        // float y_dot_setpoint = receiver::y_dot - 1500;
        // float z_dot_setpoint = receiver::z_dot - 1500;

        // x_dot_setpoint = map(x_dot_setpoint, -500, 500, -max_vel, max_vel);
        // y_dot_setpoint = map(y_dot_setpoint, -500, 500, -max_vel, max_vel);
        // z_dot_setpoint = map(z_dot_setpoint, -500, 500, -max_vel, max_vel);

        // outputs::x_dot = x_dot_controller.calculate(x_dot_setpoint, state_estimator.vx);
        // outputs::y_dot = y_dot_controller.calculate(y_dot_setpoint, state_estimator.vy);
        // outputs::z_dot = z_dot_controller.calculate(z_dot_setpoint, state_estimator.vz);

        attitude_control();
    }

    void reset_controllers()
    {
        // roll_controller.reset();
        // pitch_controller.reset();
        // yaw_controller.reset();


        roll_rate_controller.reset();
        pitch_rate_controller.reset();
        yaw_rate_controller.reset();
        roll_angle_controller.reset();
        pitch_angle_controller.reset();
    }
} // namespace control