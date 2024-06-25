#ifndef WEBSOCKET_HPP
#define WEBSOCKET_HPP

#include <WebSocketsServer.h>


namespace websocket
{
    extern WebSocketsServer webSocket;

    void setup();
    void event(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
    void send_data(uint8_t signature, uint32_t payload1, uint32_t payload2, int32_t payload3, int32_t payload4, float payload5, float payload6);
}

#endif // WEBSOCKET_HPP
