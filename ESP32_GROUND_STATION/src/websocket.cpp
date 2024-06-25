#include "websocket.hpp"


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
            int inputValue = atoi((char *)payload);
            Serial.printf("Received integer: %d\n", inputValue);
            webSocket.sendTXT(num, payload);
            break;
        }
    }

    void send_data(uint8_t signature, uint32_t payload1, uint32_t payload2, int32_t payload3, int32_t payload4, float payload5, float payload6)
    {
        String message = String(signature) + "," + String(payload1) + "," + String(payload2) + "," + String(payload3) + "," + String(payload4) + "," + String(payload5) + "," + String(payload6);
        webSocket.broadcastTXT(message);
    }
}
