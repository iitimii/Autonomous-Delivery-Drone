#ifndef WEBSOCKET_HPP
#define WEBSOCKET_HPP

#include <WebSocketsServer.h>


namespace websocket
{
    extern WebSocketsServer webSocket;

    void setup();
    void event(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
    void send_data();
}

#endif // WEBSOCKET_HPP
