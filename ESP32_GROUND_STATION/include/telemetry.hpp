#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>



namespace telemetry
{
  struct RadioData
  {
    uint8_t signature;
    uint32_t payload1;
    uint32_t payload2;
    int32_t payload3;
    int32_t payload4;
    float payload5;
    float payload6;
  };

  extern RF24 radio;
  extern const byte addresses[][6];
  extern RadioData data_tx;
  extern RadioData data_rx;

  void setup();
}

#endif // TELEMETRY_HPP
