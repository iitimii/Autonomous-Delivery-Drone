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
#include <Motors.hpp>
#include <KalmanFilter.hpp>

PIDController PID_roll_vel(kp_roll_vel, ki_roll_vel, kd_roll_vel);
PIDController PID_pitch_vel(kp_pitch_vel, ki_pitch_vel, kd_pitch_vel);
PIDController PID_yaw_vel(kp_yaw_vel, ki_yaw_vel, kd_yaw_vel);

PIDController PID_roll_ang(kp_roll_ang, ki_roll_ang, kd_roll_ang);
PIDController PID_pitch_ang(kp_pitch_ang, ki_pitch_ang, kd_pitch_ang);

KalmanFilter1d kf_roll;
KalmanFilter1d kf_pitch;

void setup()
{
    battery_setup();
    pinMode(NRF_IRQ_PIN, INPUT_PULLUP);
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
    blink_led();
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
    calculate_angle_comp();
    // get_acc_angle();

    // roll_angle = kf_roll.calculate(roll_angle_acc, roll_velocity);
    // pitch_angle = kf_pitch.calculate(pitch_angle_acc, pitch_velocity);

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
        motor_bl = throttle - pid_pitch_vel_output + pid_roll_vel_output + pid_yaw_vel_output - 10; // BL CCW

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
    receive_telemetry();
    while (micros() - loop_timer < 4000)
        ;
}
