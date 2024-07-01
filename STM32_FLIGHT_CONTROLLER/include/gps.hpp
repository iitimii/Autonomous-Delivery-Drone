#ifndef GPS_HPP
#define GPS_HPP
#include <Arduino.h>

namespace gps
{
    float latitude;
    float longitude;
    float declination = -1.4;
    uint8_t num_satelites;
    uint8_t fix_type;
}


#endif // !GPS_HPP