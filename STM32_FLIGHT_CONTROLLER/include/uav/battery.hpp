#ifndef BATTERY_HPP
#define BATTERY_HPP
#include <Arduino.h>

namespace battery
{
    extern int pin;
    extern float low;
    extern float voltage;
    extern uint16_t percentage;
    
    void setup();
    void read();
}

#endif // !BATTERY_HPP