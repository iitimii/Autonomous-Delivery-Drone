#ifndef COMPASS_HPP
#define COMPASS_HPP
#include <Arduino.h>

namespace compass
{
    float compass_heading;
    int16_t heading_lock;
    uint8_t compass_address = 0x1E;
    uint8_t baro_address = 0x76;
}


#endif // !COMPASS_HPP