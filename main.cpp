#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <baro.h>
#include <gps.h>
#include <gyro.h>
#include <telemetry.h>
#include <receiver_ppm.h>


#define STM32_board_LED PC13 

TwoWire HWire (PB3, PB10); 
HardwareSerial gpsSerial(PA12, PA11);
 

void setup() {
  HWire.setClock(400000);

  timer_setup()
}
 
void loop()
{
  loop_timer = micros() + 4000;

  while (loop_timer > micros());
}