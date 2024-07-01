#include <Arduino.h>
#include "i2c_utils.hpp"
#include "drone.hpp"
#include "flight.hpp"
#include "gyro.hpp"
#include "telemetry.hpp"
#include "receiver.hpp"
#include "led.hpp"
#include "pid_controller.hpp"
#include "battery.hpp"
#include "motors.hpp"
#include "eeprom.hpp"

void setup()
{
    battery::setup();
    led::setup();
    i2c::setup();
    telemetry::setup();
    receiver::setup();
    motors::setup();

    gyro::setup();
    gyro::calibrate();
    battery::read();
}

void loop()
{
    drone::loop_time = micros();
    drone::loop_time_prev = micros();
    gyro::read();
    gyro::calculate_attitude();

    if (channel_7 > 1500)
        armed = 1;
    if (channel_7 < 1500 && channel_3 < 1050)
        armed = 0, start = 0;
    if (armed == 1 && channel_3 < 1050 && start == 0)
        start = 1;

    if (start == 1 && channel_3 < 1050 && channel_1 > 1400 && channel_1 < 1600 && channel_2 > 1400 && channel_2 < 1600 && channel_4 > 1400 && channel_4 < 1600)
    {
        start = 2;
        pitch_angle = pitch_angle_acc;
        roll_angle = roll_angle_acc;

        PID_roll_vel.reset();
        PID_pitch_vel.reset();
        PID_yaw_vel.reset();
        PID_roll_ang.reset();
        PID_pitch_ang.reset();
    }

    battery_voltage = battery_voltage * 0.92 + get_voltage() * 0.08;
    if (battery_voltage < low_battery_warning && error == 0)
        error = 7;

    throttle = channel_3;
    if (start == 2)
    {
        roll_ang_setpoint = PID_roll_ang.channel_setpoint(channel_1);
        roll_ang_setpoint /= 10; // Max angle 50 degrees
        pitch_ang_setpoint = PID_pitch_ang.channel_setpoint(channel_2);
        pitch_ang_setpoint /= 10;
        yaw_vel_setpoint = PID_yaw_vel.channel_setpoint(channel_4);

        pid_roll_ang_output = PID_roll_ang.calculate(roll_ang_setpoint, roll_angle);
        pid_pitch_ang_output = PID_pitch_ang.calculate(pitch_ang_setpoint, pitch_angle);

        pid_roll_vel_output = PID_roll_vel.calculate(pid_roll_ang_output, roll_velocity_lpf);
        pid_pitch_vel_output = PID_pitch_vel.calculate(pid_pitch_ang_output, pitch_velocity_lpf);
        pid_yaw_vel_output = PID_yaw_vel.calculate(yaw_vel_setpoint, yaw_velocity_lpf);

        if (throttle > 1800)
            throttle = 1800;
        motor_fr = throttle + pid_pitch_vel_output - pid_roll_vel_output + pid_yaw_vel_output; // FR CCW
        motor_fl = throttle + pid_pitch_vel_output + pid_roll_vel_output - pid_yaw_vel_output; // FL CW
        motor_br = throttle - pid_pitch_vel_output - pid_roll_vel_output - pid_yaw_vel_output; // BR CW
        motor_bl = throttle - pid_pitch_vel_output + pid_roll_vel_output + pid_yaw_vel_output; // BL CCW
       motors::running();

        
    }

    else
    {
    motors::off();
    }

    telemetry::send();


    drone::loop_time_actual = micros() - drone::loop_time_prev;
    if (micros() - drone::loop_time > 4050)
        drone::error = 5;
    while (micros() - drone::loop_time < 4000)
        ;
}
