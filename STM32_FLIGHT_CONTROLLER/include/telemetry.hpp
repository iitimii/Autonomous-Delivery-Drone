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

    extern bool ack;
    extern uint64_t count;
    extern volatile bool dataReceived;

    extern int irq_pin;

    void setup();
    void select_data();
    inline void send(RadioData &data);
    //   void IRAM_ATTR interrupt();
    void interrupt();
    void loop();
    // void read(uint32_t loop_time);
}

#endif // TELEMETRY_HPP
