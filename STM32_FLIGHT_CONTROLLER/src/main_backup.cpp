// #include <Arduino.h>
// #include <SPI.h>
// #include <Wire.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <RF24_config.h>
// #include <variables.hpp>
// #include <nrf.hpp>
// #include <baro.hpp>
// #include <gps.hpp>
// #include <gyro.hpp>
// #include <telemetry.hpp>
// #include <receiver_ppm.hpp>
// #include <receiver_ibus.hpp>
// #include <compass.hpp>
// #include <calibration.hpp>
// #include <led_control.hpp>
// #include <calc_PID.hpp>
// #include <remote_drop_control.hpp>
// #include <return_to_home.hpp>
// #include <startstop_takeoff.hpp>
// #include <vertical_acc_calculation.hpp>
// #include <vector>


// void setup()
// {
//   HWire.setClock(400000);
//   HWire.begin(); // Start the I2C as master

//   pinMode(BAT_VOLTAGE_PIN, INPUT);
//   pinMode(STM32_board_LED, OUTPUT);
//   pinMode(RED_LED_PIN, OUTPUT);
//   pinMode(GREEN_LED_PIN, OUTPUT);
//   pinMode(BUZZER_PIN, OUTPUT);

//   onled(LOW);

//   delay(50);
//   telem_setup();
//   delay(50);
//   timer_setup();
//   delay(50);

//   // Check if the MPU-6050 is responding.
//   HWire.beginTransmission(gyro_address); // Start communication with the MPU-6050.
//   error = HWire.endTransmission();       // End the transmission and register the exit status.
//   while (error != 0)
//   { // Stay in this loop because the MPU-6050 did not respond.
//     error = 1;
//     send_telemetry();                      // Set the error status to 1.
//     error_signal();                        // Show the error via the red LED.
//     HWire.beginTransmission(gyro_address); // Start communication with the MPU-6050.
//     error = HWire.endTransmission();       // End the transmission and register the exit status.
//     delay(4);                              // Simulate a 250Hz refresch rate as like the main loop.
//   }

//   delay(50);
//   gyro_setup(); // Initiallize the gyro and set the correct registers.

//    for (count_var = 0; count_var < 1250; count_var++) {        //1250 loops of 4 microseconds = 5 seconds
//        if (count_var % 125 == 0) {                               //Every 125 loops (500ms).
//          digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));                   //Change the led status.
//        }
//        delay(4);                                                 //Delay 4 microseconds
//      }
//      count_var = 0;                                              //Set start back to 0.

//      calibrate_gyro();                                             //Calibrate the gyro offset.

//      //Wait until the receiver is active.
//   // while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990)  {
//   //   error = 2;                                                  //Set the error status to 3.
//   //   error_signal();                                             //Show the error via the red LED.
//   //   delay(4);
//   // }
//   error = 0;    

//   //The STM32 uses a 12 bit analog to digital converter.
//   //analogRead => 0 = 0V ..... 4095 = 3.3V
//   //The voltage divider (1k & 10k) is 1:11.
//   //analogRead => 0 = 0V ..... 4095 = 36.3V
//   //36.3 / 4095 = 112.81.
//   battery_voltage = (float)analogRead(BAT_VOLTAGE_PIN) / 112.81;
//   onled(HIGH);
// }

// void loop()
// {
//   loop_timer = micros();
//   loop_time_prev = micros();

//   read_gyro();

//   // Gyro angle calculations
//   // 0.0000611 = 1 / (250Hz / 65.5)
//   angle_pitch += (float)gyro_pitch * 0.0000611; // Calculate the traveled pitch angle and add this to the angle_pitch variable.
//   angle_roll += (float)gyro_roll * 0.0000611;   // Calculate the traveled roll angle and add this to the angle_roll variable.

//   // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
//   angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066); // If the IMU has yawed transfer the roll angle to the pitch angel.
//   angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066); // If the IMU has yawed transfer the pitch angle to the roll angel.

//   // Accelerometer angle calculations
//   acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); // Calculate the total accelerometer vector.

//   if (abs(acc_y) < acc_total_vector)
//   {                                                                   // Prevent the asin function to produce a NaN.
//     angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296; // Calculate the pitch angle.
//   }
//   if (abs(acc_x) < acc_total_vector)
//   {                                                                  // Prevent the asin function to produce a NaN.
//     angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296; // Calculate the roll angle.
//   }

//   angle_pitch = angle_pitch * 0.998 + angle_pitch_acc * 0.002; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
//   angle_roll = angle_roll * 0.998 + angle_roll_acc * 0.002;    // Correct the drift of the gyro roll angle with the accelerometer roll angle.


//   if (channel_7 > 1500) armed = 1;
//   if (channel_7 < 1500 && channel_3 < 1050) armed = 0, start = 0;

//   if (armed == 1 && channel_3 < 1050 && start == 0) start = 1;

//   if (start == 1 && channel_3 < 1050 && channel_1 > 1400 && channel_1 < 1600 && channel_2 > 1400 && channel_2 < 1600 && channel_4 > 1400 && channel_4 < 1600)
//   {
//     start = 2;

//     angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
//     angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

//     //Reset the PID controllers for a bumpless start.
//     pid_i_mem_roll = 0;
//     pid_last_roll_d_error = 0;
//     pid_i_mem_pitch = 0;
//     pid_last_pitch_d_error = 0;
//     pid_i_mem_yaw = 0;
//     pid_last_yaw_d_error = 0;
//   }

//    // calculate_pid();

//    //The battery voltage is needed for compensation.
//   //A complementary filter is used to reduce noise.
//   //1410.1 = 112.81 / 0.08.
//   battery_voltage = battery_voltage * 0.92 + ((float)analogRead(BAT_VOLTAGE_PIN) / 1410.1);
//   //Turn on the led if battery voltage is to low. In this case under 10.0V
//   if (battery_voltage < 10.0 && error == 0)error = 7;

//   throttle = channel_3;                                                            //We need the throttle signal as a base signal.

//   if (start == 2) {                                                                //The motors are started.
//     if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.
//     esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
//     esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
//     esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
//     esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

//     if (esc_1 < 1100) esc_1 = 1100;                                                //Keep the motors running.
//     if (esc_2 < 1100) esc_2 = 1100;                                                //Keep the motors running.
//     if (esc_3 < 1100) esc_3 = 1100;                                                //Keep the motors running.
//     if (esc_4 < 1100) esc_4 = 1100;                                                //Keep the motors running.

//     if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
//     if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
//     if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
//     if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.
//   }

//   else {
//     esc_1 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
//     esc_2 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
//     esc_3 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
//     esc_4 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
//   }

//   set_esc_pwm();
//   send_telemetry();
//   if (micros() - loop_timer > 4050)error = 5;                                      //Turn on the LED if the loop time exceeds 4050us.
//   loop_time_actual = micros() - loop_time_prev;

// }