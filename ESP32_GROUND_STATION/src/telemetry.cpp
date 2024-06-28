#include "telemetry.hpp"

namespace telemetry
{
    int irq_pin = 17;
    RF24 radio(21, 5);
    const byte addresses[][6] = {"00001", "00002"};
    RadioData data_tx;
    RadioData data_rx;
    volatile bool dataReceived = false;
    uint64_t count = 0;

    void setup()
    {
        radio.begin();
        radio.setChannel(120);
        radio.setDataRate(RF24_250KBPS);
        radio.setPALevel(RF24_PA_MIN);
        radio.setAutoAck(false);
        radio.disableAckPayload();
        radio.disableDynamicPayloads();
        radio.openReadingPipe(0, addresses[0]);
        radio.openWritingPipe(addresses[1]);
        radio.startListening();
        radio.maskIRQ(1, 1, 0);
        // pinMode(17, INPUT);
        attachInterrupt(digitalPinToInterrupt(irq_pin), interrupt, FALLING);
    }

    inline void send(RadioData &data)
    {
        radio.stopListening();
        radio.write(&data, sizeof(RadioData));
        radio.startListening();
    }

    void IRAM_ATTR interrupt()
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
            ++count;
            Serial.println(count);
        }
    }

    // void read(uint32_t loop_time)
    // {
    //     while (!radio.available() && ((micros() - loop_time) < 4300));
    //     // if (radio.available())
    //     radio.read(&data_rx, sizeof(RadioData));
    // }

}
