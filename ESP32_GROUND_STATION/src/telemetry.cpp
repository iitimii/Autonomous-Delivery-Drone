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
        radio.setDataRate(RF24_2MBPS);
        radio.setPALevel(RF24_PA_MAX);
        radio.setAutoAck(false);
        radio.disableDynamicPayloads();
        radio.maskIRQ(1, 1, 0);
        attachInterrupt(digitalPinToInterrupt(irq_pin), interrupt, FALLING);
        radio.openReadingPipe(0, addresses[0]);
        radio.openWritingPipe(addresses[1]);
        radio.startListening();
    }

    void send()
    {
        radio.stopListening();
        radio.write(&data_tx, sizeof(data_tx));
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
                Serial.println(data_rx.signature);
            }
        }
    }
}