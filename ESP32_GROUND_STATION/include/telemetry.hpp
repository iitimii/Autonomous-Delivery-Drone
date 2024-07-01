#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>


namespace telemetry
{
  struct RadioData
    {
        uint8_t signature;
        int16_t payload1;
        int16_t payload2;
        int16_t payload3;
        float payload4;
        float payload5;
        float payload6;
        float payload7;
        float payload8;
        float payload9;
    };

  extern RF24 radio;
  extern const byte addresses[][6];
  extern const uint8_t address_tx;
  extern const uint8_t address_rx;
  extern RadioData data_tx;
  extern RadioData data_rx;

  extern bool ack;
  extern uint64_t count;
  extern volatile bool dataReceived;

  extern int irq_pin;

  void setup();
  void send();
  void IRAM_ATTR interrupt();
  void loop();
}

#endif // TELEMETRY_HPP
