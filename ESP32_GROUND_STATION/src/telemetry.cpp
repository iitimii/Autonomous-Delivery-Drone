#include "telemetry.hpp"

RF24 telemetry::radio(21, 5);
const byte telemetry::addresses[][6] = {"00001", "00002"};
telemetry::RadioData telemetry::data_tx;
telemetry::RadioData telemetry::data_rx;

void telemetry::setup() {
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
}

// Implement other telemetry-related functions here
