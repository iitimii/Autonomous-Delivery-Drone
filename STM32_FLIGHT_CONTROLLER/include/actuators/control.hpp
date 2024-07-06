#include "controllers/pid.hpp"

namespace control
{
    void calculate();
    void reset_controllers();

    extern PIDGain roll_rate_gain;
    extern PIDGain pitch_rate_gain;
    extern PIDGain yaw_rate_gain;
    extern PIDGain roll_angle_gain;
    extern PIDGain pitch_angle_gain;

    extern PIDController roll_rate_controller;
    extern PIDController pitch_rate_controller;
    extern PIDController yaw_rate_controller;
    extern PIDController roll_angle_controller;
    extern PIDController pitch_angle_controller;

} // namespace control