#include "websocket.hpp"
#include "telemetry.hpp"
#include <ArduinoJSON.h>

namespace websocket
{
    WebSocketsServer webSocket(80);

    void setup()
    {
        webSocket.begin();
        webSocket.onEvent(event);
    }

    void event(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
    {
        switch (type)
        {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
        {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            webSocket.sendTXT(num, "Connected");
        }
        break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);

            StaticJsonDocument<256> doc;

            DeserializationError error = deserializeJson(doc, payload);

            if (error)
            {
                Serial.print(F("deserializeJson() failed: "));
                Serial.println(error.f_str());
                return;
            }

            const char *type = doc["type"];
            const char *message = doc["message"];
            // telemetry::ack = false;

            if (strcmp(type, "PID") == 0)
            {
                Serial.printf("Received PID message: %s\n", message);
                // message in this format P,A,R,5
                telemetry::data_tx.signature = 'P';
                telemetry::data_tx.payload1 = message[0];
                telemetry::data_tx.payload2 = message[2];
                telemetry::data_tx.payload3 = message[4];
                telemetry::data_tx.payload5 = message[6];
            }

            else if (strcmp(type, "custom") == 0)
            {
                Serial.printf("Received custom message: %s\n", message);
                telemetry::data_tx.signature = 'C';
            }
            else
            {
                Serial.printf("Unknown message type: %s\n", type);
            }

            telemetry::send(telemetry::data_tx);

            break;
        }
    }

    void send_data(uint8_t signature, uint32_t payload1, uint32_t payload2, int32_t payload3, int32_t payload4, float payload5, float payload6)
    {
        String message = String(signature) + "," + String(payload1) + "," + String(payload2) + "," + String(payload3) + "," + String(payload4) + "," + String(payload5) + "," + String(payload6);
        webSocket.broadcastTXT(message);
    }
}
