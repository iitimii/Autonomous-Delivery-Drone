#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <variables.h>
#include<nrf.h>
#include <IBusBM.h>
#include <baro.h>
#include <gps.h>
#include <gyro.h>
#include <telemetry.h>
#include <receiver_ppm.h>
#include <receiver_ibus.h>
#include <compass.h>
#include <calibration.h>
#include <led_control.h>
#include <calc_PID.h>
#include <remote_drop_control.h>
#include <return_to_home.h>
#include <startstop_takeoff.h>
#include <vertical_acc_calculation.h>

 

void setup() {
  HWire.setClock(400000);
  HWire.begin();                                                //Start the I2C as master

  pinMode(BAT_VOLTAGE_PIN, INPUT);
  pinMode(STM32_board_LED, OUTPUT);                            
  pinMode(RED_LED_PIN, OUTPUT);     
  pinMode(GREEN_LED_PIN, OUTPUT);     
  pinMode(BUZZER_PIN, OUTPUT);      

  onled(LOW);
  green_led(LOW);                                               
  red_led(HIGH); 

  delay(50);
  telem_setup();
  delay(50);
  timer_setup();
  delay(50);


    //Check if the MPU-6050 is responding.
  // HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  // error = HWire.endTransmission();                              //End the transmission and register the exit status.
  // while (error != 0) {                                          //Stay in this loop because the MPU-6050 did not respond.
  //   error = 1;      
  //   send_telemetry();                                            //Set the error status to 1.
  //   error_signal();                                             //Show the error via the red LED.
  //   delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  // }

  //   //Check if the compass is responding.
  // HWire.beginTransmission(compass_address);                     //Start communication with the HMC5883L.
  // error = HWire.endTransmission();                              //End the transmission and register the exit status.
  // while (error != 0) {                                          //Stay in this loop because the HMC5883L did not responde.
  //   error = 2;     
  //   // send_telemetry();                                             //Set the error status to 2.
  //   error_signal();                                             //Show the error via the red LED.
  //   delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  // }

  // //  Check if the LPS22HB barometer is responding.
  // HWire.beginTransmission(barometer_address);                      //Start communication with the LPS22HB.
  // error = HWire.endTransmission();                              //End the transmission and register the exit status.
  // while (error != 0) {                                          //Stay in this loop because the LPS22HB did not responde.
  //   error = 3;        
  //   // send_telemetry();                                          //Set the error status to 3.
  //   error_signal();                                             //Show the error via the red LED.
  //   delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  // }

  //       // Check if the EEPROM is responding.
  // HWire.beginTransmission(eeprom_address);                      //Start communication with the EEPROM.
  // error = HWire.endTransmission();                              //End the transmission and register the exit status.
  // while (error != 0) {                                          //Stay in this loop because the EEPROM did not responde.
  //   error = 4;        
  //   // send_telemetry();                                          //Set the error status to 2.
  //   error_signal();                                             //Show the error via the red LED.
  //   delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  // }


  delay(50);
  gyro_setup();                                                 //Initiallize the gyro and set the correct registers.

  for (count_var = 0; count_var < 1250; count_var++) {        //1250 loops of 4 microseconds = 5 seconds
      if (count_var % 125 == 0) {                               //Every 125 loops (500ms).
        digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));                   //Change the led status.
      }
      delay(4);                                                 //Delay 4 microseconds
    }
    count_var = 0;                                              //Set start back to 0.

    calibrate_gyro();                                             //Calibrate the gyro offset.



  while (channel_1 < 900 || channel_2 < 900 || channel_3 < 900 || channel_4 < 900)  {
    error = 5;                                                  //Set the error status to 3.
    error_signal();                                             //Show the error via the red LED.
    send_telemetry();
    delay(4);
  }
  error = 0;                                                    //Reset the error status to 0.

   //Wait until the throtle is set to the lower position.
  while (channel_3 < 900 || channel_3 > 1050)  {
    error = 6;                                                  //Set the error status to 4.
    error_signal();                                             //Show the error via the red LED.
    send_telemetry();
    delay(4);
  }
  error = 0;      

  // //When everything is done, turn off the led.
  red_led(LOW);                                                 //Set output  low.

  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 4095 = 36.3V
  //36.3 / 4095 = 112.81.
  battery_voltage = (float)analogRead(BAT_VOLTAGE_PIN) / 112.81;
  green_led(HIGH);                                              //Turn on the green led.
  onled(HIGH);

  loop_timer = micros();
}

 
void loop()
{
  loop_time_prev = micros();
  error_signal();                                                                  //Show the errors via the red LED.
  read_gyro();                                                                 //Read the gyro and accelerometer data.

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angle.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angle.

  
    //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                           //Calculate the pitch angle correction.
  roll_level_adjust = angle_roll * 15;                                             //Calculate the roll angle correction.

  if (!auto_level) {                                                               //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                        //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                         //Set the roll angle correcion to zero.
  }

  //For starting the motors: throttle low and yaw left (step 1).
  // if (channel_3 < 1050 && channel_4 < 1050)start = 1;
  if (channel_6 > 1500)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && channel_3 < 1050 && channel_4 > 1400) {
    start = 2;

    green_led(LOW);                                                                //Turn off the green led.

    angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;

  //     if (channel_8 < 1700)auto_level = true;
  // else auto_level = false;
  }
  //Stopping the motors: throttle low and SWA up.
  if (start == 2 && channel_3 < 1050 && channel_6 < 1500) {
    start = 0;
    // green_led(HIGH);                                                               //Turn on the green led.
  }

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of dividing by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_1 > 1508)pid_roll_setpoint = channel_1 - 1508;
  else if (channel_1 < 1492)pid_roll_setpoint = channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_2 > 1508)pid_pitch_setpoint = channel_2 - 1508;
  else if (channel_2 < 1492)pid_pitch_setpoint = channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_3 > 1050) { //Do not yaw when turning off the motors.
    if (channel_4 > 1508)pid_yaw_setpoint = (channel_4 - 1508) / 3.0;
    else if (channel_4 < 1492)pid_yaw_setpoint = (channel_4 - 1492) / 3.0;
  }

  calculate_pid();


//The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(BAT_VOLTAGE_PIN) / 1410.1);
  //Turn on the led if battery voltage is to low. In this case under 10.0V
  if (battery_voltage < 10.0 && error == 0)error = 1;

  throttle = channel_3;                                                            //We need the throttle signal as a base signal.

  if (start == 2) {                                                                //The motors are started.
    if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

    if (esc_1 < 1100) esc_1 = 1100;                                                //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                                //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                                //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                                //Keep the motors running.

    if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.
  }

  else {
    esc_1 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
  }

  set_esc_pwm();
  send_telemetry();  
  loop_time_count = micros() - loop_time_prev;
  while (micros() - loop_timer < 4000);                                        
  loop_timer = micros();  
}