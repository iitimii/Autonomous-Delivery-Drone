#include <Arduino.h>
#include "wifi.hpp"
#include "websocket.hpp"
#include "telemetry.hpp"
#

uint32_t loop_time;

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
    loop_time = micros();
    telemetry::loop();
    
    websocket::send_data();
    websocket::webSocket.loop();

    while (micros() - loop_time < 2000)
        ;
}
