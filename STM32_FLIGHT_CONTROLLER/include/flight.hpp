#ifndef FLIGHT_HPP
#define FLIGHT_HPP
#include <Arduino.h>

namespace flight
{

    extern uint8_t start, armed, ready;
    extern uint8_t mode;
    extern uint8_t takeoff_detected, manual_altitude_change;
    extern bool auto_level;

}

#endif // !FLIGHT_HPP