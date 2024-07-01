#ifndef I2C_UTILS_HPP
#define I2C_UTILS_HPP
#include <Arduino.h>
#include <Wire.h>



namespace i2c
{
    extern TwoWire HWire;
    void setup();
    void check(const uint8_t& address, const uint8_t& error_code);
    void scan();
    void write(uint8_t address, uint8_t reg, uint8_t value);
}



#endif // !I2C_UTILS_HPP