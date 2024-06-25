#include <Arduino.h>
#include "wifi.hpp"
#include "websocket.hpp"
#include "telemetry.hpp"

uint32_t loop_timer;

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    if (wifi::connect(wifi::ssid, wifi::password) == 0)
        digitalWrite(LED_BUILTIN, HIGH);

    telemetry::setup();
    websocket::setup();
}

void loop()
{
    loop_timer = micros();

    telemetry::radio.startListening();
    while (!telemetry::radio.available())
    {
        websocket::webSocket.loop();
        while (micros() - loop_timer < 2000)
            ;
    }
    telemetry::radio.read(&telemetry::data_rx, sizeof(telemetry::RadioData));
    websocket::send_data(telemetry::data_rx.signature, telemetry::data_rx.payload1, telemetry::data_rx.payload2, telemetry::data_rx.payload3, telemetry::data_rx.payload4, telemetry::data_rx.payload5, telemetry::data_rx.payload6);

    delay(1000);
    telemetry::radio.stopListening();
    telemetry::radio.write(&telemetry::data_tx, sizeof(telemetry::RadioData));

    websocket::webSocket.loop();
    while (micros() - loop_timer < 4000)
        ;
}
