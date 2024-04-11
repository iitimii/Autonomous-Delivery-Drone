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
#include <calc_PID.hpp>

void setup()
{
  onled(HIGH);
  ready = 0;
  HWire.setClock(400000);
  HWire.begin();

  pinMode(BAT_VOLTAGE_PIN, INPUT_ANALOG);
  pinMode(STM32_board_LED, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  delay(50);

  telem_setup();
  delay(50);
  timer_setup();
  delay(50);

  HWire.beginTransmission(gyro_address);
  error = HWire.endTransmission();
  while (error != 0)
  {
    error = 1;
    send_telemetry();
    delay(4);
    HWire.beginTransmission(gyro_address);
    error = HWire.endTransmission();
  }
  delay(50);
// TODO: Implement the following function for checking i2c devices
  gyro_setup();
  for (count_var = 0; count_var < 1250; ++count_var)
  {
    if (count_var % 125 == 0)
    {
      digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));
    }
    delay(4);
  }
  count_var = 0;
  calibrate_gyro();

  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 800 || channel_4 < 990)
  {
    error = 2;
    send_telemetry();
    delay(4);
  }

  battery_voltage = static_cast<float>(analogRead(BAT_VOLTAGE_PIN)) / 112.81;
  error = 0;
  ready = 1;
  onled(HIGH);
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

  angle_pitch = angle_pitch * 0.996 + angle_pitch_acc * 0.004;
  angle_roll = angle_roll * 0.996 + angle_roll_acc * 0.004;

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

  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(BAT_VOLTAGE_PIN) / 1410.1);
  // if (battery_voltage < 10.0 && error == 0)error = 7;

  throttle = channel_3;
  if (start == 2)
  {
    if (throttle > 1800)
      throttle = 1800;
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

    if (esc_1 < 1100)
      esc_1 = 1100;
    else if (esc_1 > 2000)
      esc_1 = 2000;

    if (esc_2 < 1100)
      esc_2 = 1100;
    else if (esc_2 > 2000)
      esc_2 = 2000;

    if (esc_3 < 1100)
      esc_3 = 1100;
    else if (esc_3 > 2000)
      esc_3 = 2000;

    if (esc_4 < 1100)
      esc_4 = 1100;
    else if (esc_4 > 2000)
      esc_4 = 2000;
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



