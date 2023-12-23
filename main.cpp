#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <variables.h>
#include <IBusBM.h>
#include <EEPROM.h>
#include "baro.h"
#include "gps.h"
#include "gyro.h"
#include "telemetry.h"
#include "receiver_ppm.h"
#include "receiver_ibus.h"
#include "compass.h"
#include "error_handle.h"
#include "calibration.h"
#include "state_signal.h"

 

void setup() {
  HWire.setClock(400000);
  HWire.begin();                                                //Start the I2C as master
  pinMode(STM32_board_LED, OUTPUT);                             //This is the LED on the STM32 board. Used for GPS indication.
  pinMode(RED_LED_PIN, OUTPUT);     
  pinMode(GREEN_LED_PIN, OUTPUT);     
  pinMode(BUZZER_PIN, OUTPUT);                                 
  digitalWrite(STM32_board_LED, HIGH);

  telem_setup();
  delay(50);
  timer_setup();
  delay(50);
  gps_setup();
  delay(50);

   //    //Check if the EEPROM is responding.
  // HWire.beginTransmission(LPS22HB_ADDRESS);                      //Start communication with the EEPROM.
  // error = HWire.endTransmission();                              //End the transmission and register the exit status.
  // while (error != 0) {                                          //Stay in this loop because the EEPROM did not responde.
  //   error = 5;        
  //   send_telemetry();                                          //Set the error status to 2.
  //   // error_signal();                                             //Show the error via the red LED.
  //   delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  // }

    //Check if the MPU-6050 is responding.
  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MPU-6050 did not respond.
    error = 1;      
    send_telemetry();                                            //Set the error status to 1.
    // error_signal();                                             //Show the error via the red LED.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

    //Check if the compass is responding.
  HWire.beginTransmission(compass_address);                     //Start communication with the HMC5883L.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the HMC5883L did not responde.
    error = 2;     
    send_telemetry();                                             //Set the error status to 2.
    // error_signal();                                             //Show the error via the red LED.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

  //  Check if the LPS22HB barometer is responding.
  HWire.beginTransmission(LPS22HB_ADDRESS);                      //Start communication with the LPS22HB.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the LPS22HB did not responde.
    error = 3;        
    send_telemetry();                                          //Set the error status to 3.
    // error_signal();                                             //Show the error via the red LED.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }



  baro_setup();                                                 //Initiallize the baro and set the correct registers.
  delay(50);
  gyro_setup();                                                 //Initiallize the gyro and set the correct registers.
  delay(50);
  // setup_compass();                                              //Initiallize the compass and set the correct registers.
  delay(50);


  // read_compass();                                               //Read and calculate the compass data.
  angle_yaw = actual_compass_heading;                           //Set the initial compass heading.

    // Create a 5 second delay before calibration.
  for (count_var = 0; count_var < 1250; count_var++) {          //1250 loops of 4 microseconds = 5 seconds.
    if (count_var % 125 == 0) {                                 //Every 125 loops (500ms).
      digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));                     //Change the led status.
      //Send callibration warning to GS
      digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
    }
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }
  count_var = 0;                                                //Set start back to 0.\\\\\\

  calibrate_gyro();                                             //Calibrate the gyro offset.

  //Wait until the receiver is active.
  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990)  {
    error = 4;                                                  //Set the error status to 4.
    // error_signal(); 
    send_telemetry();                                            //Show the error via the red LED.
    delay(4);                                                   //Delay 4ms to simulate a 250Hz loop
  }
  error = 0;                                                    //Reset the error status to 0.
   
  // read battery voltage
  battery_voltage = (float)analogRead(PB1) / 112.81;

    //The LPS22HB needs a few readings to stabilize.
  // for (start = 0; start < 100; start++) {                       //This loop runs 100 times.
  //   read_baro();                                           //Read and calculate the barometer data.
  //   delay(4);                                                   //The main program loop also runs 250Hz (4ms per loop).
  // }
  // actual_pressure = 0;    

  // //Before starting the avarage accelerometer value is preloaded into the variables.
  // for (start = 0; start <= 24; start++)acc_z_average_short[start] = acc_z;
  // for (start = 0; start <= 49; start++)acc_z_average_long[start] = acc_z;
  // acc_z_average_short_total = acc_z * 25;
  // acc_z_average_long_total = acc_z * 50;
  // start = 0;

  // if (motor_idle_speed < 1000)motor_idle_speed = 1000;          //Limit the minimum idle motor speed to 1000us.
  // if (motor_idle_speed > 1200)motor_idle_speed = 1200;          //Limit the maximum idle motor speed to 1200us.


  loop_timer = micros();
}

 
void loop()
{
  loop_time_prev = micros();


  // if(receiver_watchdog < 750)receiver_watchdog ++;
  // if(receiver_watchdog == 750 && start == 2){
  //   channel_1 = 1500;
  //   channel_2 = 1500;
  //   channel_3 = 1500;
  //   channel_4 = 1500;
  //   error = 8;
  //   if (number_used_sats > 5){
  //     if(home_point_recorded == 1)channel_5 = 2000;
  //     else channel_5 = 1750;
  //   }
  //   else channel_5 = 1500;    
  // }


  read_gps();
  read_baro();
  set_esc_pwm();
  send_telemetry();  



  loop_time_count = micros() - loop_time_prev;
  while (micros() - loop_timer < 4000);                                        
  loop_timer = micros();  
}