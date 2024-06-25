#include <Arduino.h>
#include "wifi.hpp"
#include "websocket.hpp"
#include "telemetry.hpp"
#

uint32_t loop_timer;

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    if (wifi::connect(wifi::ssid, wifi::password) == 0 && wifi::DNS_setup() == 0)
        digitalWrite(LED_BUILTIN, HIGH);

    telemetry::setup();
    websocket::setup();
}

void loop()
{
    loop_timer = micros();
    telemetry::loop();
    websocket::send_data(telemetry::data_rx.signature, telemetry::data_rx.payload1, telemetry::data_rx.payload2, telemetry::data_rx.payload3, telemetry::data_rx.payload4, telemetry::data_rx.payload5, telemetry::data_rx.payload6);
    websocket::webSocket.loop();
    // telemetry::send(telemetry::data_tx);

    while (micros() - loop_timer < 4300)
        ;
}
