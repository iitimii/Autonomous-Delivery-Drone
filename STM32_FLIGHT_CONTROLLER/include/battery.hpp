#ifndef BATTERY_HPP
#define BATTERY_HPP
#include <Arduino.h>

namespace battery
{
    extern int pin;
    extern float low;
    extern float voltage;
    
    void setup();
    void read();
}

#endif // !BATTERY_HPP