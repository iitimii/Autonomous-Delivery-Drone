#ifndef GPS_HPP
#define GPS_HPP
#include <cstdint>
#include <Arduino.h>
#include "sensors/Ublox.h"


extern HardwareSerial gpsSerial;

class GPS
{
private:
    float latitude, longitude, altitude, vertical_speed;
    uint8_t num_satelites, fix_type;

    // uint8_t latitude_north, longitude_east, new_line_found, incomming_message[100];
    // uint16_t message_counter;

    Ublox M8_Gps;

    

public:
    GPS();
    void setup();
    void read();

    // Getters
    inline float getLatitude() const { return latitude; }
    inline float getLongitude() const { return longitude; }
    inline uint8_t getNumSats() const { return num_satelites; }
    inline uint8_t getFixType() const { return fix_type; }
    inline float getAltitude() const { return altitude; }
    inline float getVerticalSpeed() const { return vertical_speed; }
    
};

#endif // !GPS_HPP
