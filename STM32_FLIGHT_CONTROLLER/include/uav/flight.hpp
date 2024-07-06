#ifndef FLIGHT_HPP
#define FLIGHT_HPP
#include <Arduino.h>

namespace flight
{
    extern uint8_t start, mode;
    extern uint16_t armed;

    void setup();
    void update();
}

#endif // !FLIGHT_HPP