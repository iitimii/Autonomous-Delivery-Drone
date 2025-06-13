#include "controllers/pid.hpp"

namespace control
{
    void calculate();
    void reset_controllers();
    void rate_control();
    void attitude_control();
    void velocity_control();

    extern PIDGain roll_gain;
    extern PIDGain pitch_gain;
    extern PIDGain yaw_gain;

    extern PIDGain x_dot_gain;
    extern PIDGain y_dot_gain;
    extern PIDGain z_dot_gain;

    extern PIDController roll_controller;
    extern PIDController pitch_controller;
    extern PIDController yaw_controller;

    extern PIDController x_dot_controller;
    extern PIDController y_dot_controller;
    extern PIDController z_dot_controller;

    // extern PIDGain roll_rate_gain;
    // extern PIDGain pitch_rate_gain;
    // extern PIDGain yaw_rate_gain;
    
    // extern PIDGain roll_angle_gain;
    // extern PIDGain pitch_angle_gain;

    // extern PIDGain x_dot_gain;
    // extern PIDGain y_dot_gain;
    // extern PIDGain z_dot_gain;

    // extern PIDController roll_rate_controller;
    // extern PIDController pitch_rate_controller;
    // extern PIDController yaw_rate_controller;

    // extern PIDController roll_angle_controller;
    // extern PIDController pitch_angle_controller;

    // extern PIDController x_dot_controller;
    // extern PIDController y_dot_controller;
    // extern PIDController z_dot_controller;

} // namespace control