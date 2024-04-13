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

void setup()
{
    battery_setup();
    blueled(HIGH);
    ready = 0;
    HWire.setClock(400000);
    HWire.begin();
    HWire.setClock(400000);

    pinMode(BAT_VOLTAGE_PIN, INPUT_ANALOG);
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
    gyro_roll_input = (gyro_roll_input * 0.7) + ((static_cast<float>(gyro_roll) / 65.5) * 0.3);
    gyro_pitch_input = (gyro_pitch_input * 0.7) + ((static_cast<float>(gyro_pitch) / 65.5) * 0.3);
    gyro_yaw_input = (gyro_yaw_input * 0.7) + ((static_cast<float>(gyro_yaw) / 65.5) * 0.3);

    angle_pitch += static_cast<float>(gyro_pitch) * 0.0000610687;
    angle_roll += static_cast<float>(gyro_roll) * 0.0000610687;

    angle_pitch -= angle_roll * sin(static_cast<float>(gyro_yaw) * 0.00000106585);
    angle_roll += angle_pitch * sin(static_cast<float>(gyro_yaw) * 0.00000106585);

    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
    if (abs(acc_y) < acc_total_vector)
        angle_pitch_acc = asin(static_cast<float>(acc_y) / acc_total_vector) * 57.29578;
    if (abs(acc_x) < acc_total_vector)
        angle_roll_acc = asin(static_cast<float>(acc_x) / acc_total_vector) * 57.29578;

    angle_pitch = angle_pitch * 0.995 + angle_pitch_acc * 0.005;
    angle_roll = angle_roll * 0.995 + angle_roll_acc * 0.005;

    if (channel_7 > 1500)
        armed = 1;
    if (channel_7 < 1500 && channel_3 < 1050)
        armed = 0, start = 0;
    if (armed == 1 && channel_3 < 1050 && start == 0)
        start = 1;

    if (start == 1 && channel_3 < 1050 && channel_1 > 1400 && channel_1 < 1600 && channel_2 > 1400 && channel_2 < 1600 && channel_4 > 1400 && channel_4 < 1600)
    {
        start = 2;
        angle_pitch = angle_pitch_acc;
        angle_roll = angle_roll_acc;
        i_term_roll = 0;
        pid_last_roll_d_error = 0;
        i_term_pitch = 0;
        pid_last_pitch_d_error = 0;
        i_term_yaw = 0;
        pid_last_yaw_d_error = 0;
    }
    calculate_pid();

    battery_voltage = battery_voltage * 0.92 + get_voltage() * 0.08;
    if (battery_voltage < 10.0 && error == 0)
        error = 7;

    throttle = channel_3;
    if (start == 2)
    {
        if (throttle > 1800)
            throttle = 1800;
        esc_1 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw; // FR CCW
        esc_2 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw; // FL CW
        esc_3 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw; // BR CW
        esc_4 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw; // BL CCW

        if (esc_1 < 1100)
            esc_1 = 1100;
        else if (esc_1 > 1995)
            esc_1 = 1995;

        if (esc_2 < 1100)
            esc_2 = 1100;
        else if (esc_2 > 1995)
            esc_2 = 1995;

        if (esc_3 < 1100)
            esc_3 = 1100;
        else if (esc_3 > 1995)
            esc_3 = 1995;

        if (esc_4 < 1100)
            esc_4 = 1100;
        else if (esc_4 > 1995)
            esc_4 = 1995;
    }

    else
    {
        esc_1 = 1000;
        esc_2 = 1000;
        esc_3 = 1000;
        esc_4 = 1000;
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
