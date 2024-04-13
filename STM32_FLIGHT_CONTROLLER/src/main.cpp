#include <Arduino.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <variables.hpp>
#include <gyro.hpp>
#include <telemetry.hpp>
#include <receiver_ppm.hpp>
#include <calibration.hpp>
#include <led_control.hpp>
#include <PID.hpp>
#include <I2C_utils.hpp>
#include <voltage.hpp>

PIDController PID_roll_vel(kp_roll_vel, ki_roll_vel, kd_roll_vel, dt);
PIDController PID_pitch_vel(kp_pitch_vel, ki_pitch_vel, kd_pitch_vel, dt);
PIDController PID_yaw_vel(kp_yaw_vel, ki_yaw_vel, kd_yaw_vel, dt);

void setup()
{
    battery_setup();
    blueled(HIGH);
    ready = 0;
    HWire.setClock(400000);
    HWire.begin();
    HWire.setClock(400000);

    pinMode(STM32_board_LED, OUTPUT);

    telem_setup();
    timer_setup();

    check_device(gyro_address, 1);
    check_device(eeprom_address, 2);
    check_device(baro_address, 3);

    gyro_setup();
    // blink_led();
    calibrate_gyro();

    wait_for_receiver();

    battery_voltage = get_voltage();

    error = 0;
    ready = 1;
    blueled(LOW);
    send_telemetry();
}

void loop()
{
    loop_timer = micros();
    loop_time_prev = micros();
    //------------------------------------------------------------------------------
    read_gyro();
    roll_velocity_lpf = (roll_velocity_lpf * 0.7) + ((static_cast<float>(roll_velocity) / 65.5) * 0.3);
    pitch_velocity_lpf = (pitch_velocity_lpf * 0.7) + ((static_cast<float>(pitch_velocity) / 65.5) * 0.3);
    yaw_velocity_lpf = (yaw_velocity_lpf * 0.7) + ((static_cast<float>(yaw_velocity) / 65.5) * 0.3);

    pitch_angle += static_cast<float>(pitch_velocity) * 0.0000610687;
    roll_angle += static_cast<float>(roll_velocity) * 0.0000610687;

    pitch_angle -= roll_angle * sin(static_cast<float>(yaw_velocity) * 0.00000106585);
    roll_angle += pitch_angle * sin(static_cast<float>(yaw_velocity) * 0.00000106585);

    acc_resultant = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
    if (abs(acc_y) < acc_resultant)
        pitch_angle_acc = asin(static_cast<float>(acc_y) / acc_resultant) * 57.29578;
    if (abs(acc_x) < acc_resultant)
        roll_angle_acc = asin(static_cast<float>(acc_x) / acc_resultant) * 57.29578;

    pitch_angle = pitch_angle * 0.995 + pitch_angle_acc * 0.005;
    roll_angle = roll_angle * 0.995 + roll_angle_acc * 0.005;

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
    }

    
    battery_voltage = battery_voltage * 0.92 + get_voltage() * 0.08;
    if (battery_voltage < low_battery_warning && error == 0)
        error = 7;

    throttle = channel_3;
    if (start == 2)
    {
        roll_vel_setpoint = PID_roll_vel.channel_setpoint(channel_1);
        pitch_vel_setpoint = PID_pitch_vel.channel_setpoint(channel_2);
        yaw_vel_setpoint = PID_yaw_vel.channel_setpoint(channel_4);

        pid_roll_vel_output = PID_roll_vel.calculate(roll_vel_setpoint, roll_velocity_lpf);
        pid_pitch_vel_output = PID_pitch_vel.calculate(pitch_vel_setpoint, pitch_velocity_lpf);
        pid_yaw_vel_output = PID_yaw_vel.calculate(yaw_vel_setpoint, yaw_velocity_lpf);


        if (throttle > 1800)
            throttle = 1800;
        motor_fr = throttle + pid_pitch_vel_output - pid_roll_vel_output + pid_yaw_vel_output; // FR CCW
        motor_fl = throttle + pid_pitch_vel_output + pid_roll_vel_output - pid_yaw_vel_output; // FL CW
        motor_br = throttle - pid_pitch_vel_output - pid_roll_vel_output - pid_yaw_vel_output; // BR CW
        motor_bl = throttle - pid_pitch_vel_output + pid_roll_vel_output + pid_yaw_vel_output; // BL CCW

        if (motor_fr < 1100)
            motor_fr = 1100;
        else if (motor_fr > 1995)
            motor_fr = 1995;

        if (motor_fl < 1100)
            motor_fl = 1100;
        else if (motor_fl > 1995)
            motor_fl = 1995;

        if (motor_br < 1100)
            motor_br = 1100;
        else if (motor_br > 1995)
            motor_br = 1995;

        if (motor_bl < 1100)
            motor_bl = 1100;
        else if (motor_bl > 1995)
            motor_bl = 1995;
    }

    else
    {
        motor_fr = 1000;
        motor_fl = 1000;
        motor_br = 1000;
        motor_bl = 1000;
    }

    set_esc_pwm();
    //------------------------------------------------------------------------------
    loop_time_actual = micros() - loop_time_prev;
    if (micros() - loop_timer > 4050)
        error = 5;
    send_telemetry();
    while (micros() - loop_timer < 4000)
        ;
}
