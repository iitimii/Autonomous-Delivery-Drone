#include "telemetry.hpp"
#include "led.hpp"
#include "gps.hpp"
#include "drone.hpp"
#include "flight.hpp"
#include "compass.hpp"
#include "battery.hpp"
#include "gyro.hpp"
#include "baro.hpp"
#include "receiver.hpp"
#include "motors.hpp"
#include "pid_controller.hpp"

namespace telemetry
{
  int irq_pin = PA1;
  RF24 radio(PB0, PA4);
  const byte addresses[][6] = {"00001", "00002"};

  RadioData data_tx;
  RadioData data_rx;

  volatile bool dataReceived = false;

  void setup()
  {
    radio.begin();
    radio.setChannel(120);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MIN);
    radio.setAutoAck(false);
    radio.disableDynamicPayloads();
    // radio.setRetries(0, 2);
    radio.maskIRQ(1, 1, 0);
    attachInterrupt(digitalPinToInterrupt(irq_pin), interrupt, FALLING);
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(0, addresses[1]);
    radio.startListening();
  }

  void select_data()
  {
    static uint8_t telemetry_loop_counter = 0;

    switch (telemetry_loop_counter)
    {
    case 0:
      data_tx.signature = 'T';             // uint8_t
      data_tx.payload1 = drone::loop_time_actual; // int16_t
      data_tx.payload2 = drone::error;                // int16_t
      data_tx.payload3 = flight::start;                // int16_t
      data_tx.payload4 = battery::voltage;               
      data_tx.payload5 = gyro::pitch;             
      data_tx.payload6 = gyro::roll; 
      data_tx.payload7 = gyro::yaw;               
      data_tx.payload8 = gyro::pitch_accelerometer;             
      data_tx.payload9 = gyro::roll_accelerometer;              
      break;

    case 1:
      data_tx.signature = 'I';
      data_tx.payload1 = flight::armed;
      data_tx.payload2 = flight::mode;
      data_tx.payload3 = compass::heading_lock;
      data_tx.payload4 = gyro::pitch_rate;               
      data_tx.payload5 = gyro::roll_rate;             
      data_tx.payload6 = gyro::yaw_rate; 
      data_tx.payload7 = gps::latitude;               
      data_tx.payload8 = gps::longitude;             
      data_tx.payload9 = compass::compass_heading;  
      break;

    case 2:
      data_tx.signature = 'M';
      data_tx.payload1 = gps::num_satelites;
      data_tx.payload2 = gps::fix_type;
      data_tx.payload3 = 0;
      data_tx.payload4 = baro::pressure;               
      data_tx.payload5 = baro::altitude;             
      data_tx.payload6 = baro::temperature; 
      data_tx.payload7 = receiver::channels[0];               
      data_tx.payload8 = receiver::channels[1];             
      data_tx.payload9 = receiver::channels[2];  
      break;

    case 3:
      data_tx.signature = 'L';
      data_tx.payload1 = receiver::channels[3];
      data_tx.payload2 = receiver::channels[4];
      data_tx.payload3 = receiver::channels[5];
      data_tx.payload4 = receiver::channels[6];               
      data_tx.payload5 = receiver::channels[7];             
      data_tx.payload6 = receiver::channels[8]; 
      data_tx.payload7 = receiver::channels[9];               
      data_tx.payload8 = pid::output.pitch;             
      data_tx.payload9 = pid::output.roll;  
      break;

    case 4:
      data_tx.signature = 'H';
      data_tx.payload1 = motors::br;
      data_tx.payload2 = motors::bl;
      data_tx.payload3 = pid::output.throttle;
      data_tx.payload4 = pid::output.yaw;               
      data_tx.payload5 = pid::output.angle.pitch;             
      data_tx.payload6 = pid::output.angle.roll; 
      data_tx.payload7 = pid::gain.rate.pitch.kp;               
      data_tx.payload8 = pid::gain.rate.pitch.ki;             
      data_tx.payload9 = pid::gain.rate.pitch.kd;  
      break;

    case 5:
      data_tx.signature = 'N';
      data_tx.payload1 = motors::fl;
      data_tx.payload2 = motors::fr;
      data_tx.payload3 = 0;
      data_tx.payload4 = pid::gain.rate.roll.kp;               
      data_tx.payload5 = pid::gain.rate.roll.ki;             
      data_tx.payload6 = pid::gain.rate.roll.kd; 
      data_tx.payload7 = pid::gain.rate.yaw.kp;               
      data_tx.payload8 = pid::gain.rate.yaw.ki;             
      data_tx.payload9 = pid::gain.rate.yaw.kd; 
      break;

    case 6:
      data_tx.signature = 'O';
      data_tx.payload1 = 0;
      data_tx.payload2 = 0;
      data_tx.payload3 = 0;
      data_tx.payload4 = pid::gain.angle.pitch.kp;               
      data_tx.payload5 = pid::gain.angle.pitch.ki;             
      data_tx.payload6 = pid::gain.angle.pitch.kd; 
      data_tx.payload7 = pid::gain.angle.roll.kp;               
      data_tx.payload8 = pid::gain.angle.roll.ki;             
      data_tx.payload9 = pid::gain.angle.roll.kd; 
      break;

    default:
      data_tx.signature = 'E';             // uint8_t
      data_tx.payload1 = 0;                // int16_t
      data_tx.payload2 = 0;                // int16_t
      data_tx.payload3 = 0;                // int16_t
      data_tx.payload4 = 0.0f;               
      data_tx.payload5 = 0.0f;             
      data_tx.payload6 = 0.0f; 
      data_tx.payload7 = 0.0f;               
      data_tx.payload8 = 0.0f;             
      data_tx.payload9 = 0.0f;  
      break;
    }

    telemetry_loop_counter++;
    if (telemetry_loop_counter >= 7)
      telemetry_loop_counter = 0;
  }

  void send()
  {
    radio.stopListening();
    select_data();
    radio.write(&data_tx, sizeof(data_tx));
    radio.startListening();
  }

  void interrupt()
  {
    dataReceived = true;
  }

  void loop()
  {
    if (dataReceived)
    {
      dataReceived = false;
      while (radio.available())
      {
        radio.read(&data_rx, sizeof(RadioData));
      }
    }
  }

}