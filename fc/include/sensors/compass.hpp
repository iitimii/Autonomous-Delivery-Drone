#ifndef COMPASS_HPP
#define COMPASS_HPP
#include <cstdint>

#define CALIBRATION_SIGNATURE 0xAB // Signature to verify calibration data is valid
#define EEPROM_SIGNATURE_ADDR 0
#define EEPROM_OFFSET_X_ADDR 1
#define EEPROM_OFFSET_Y_ADDR 5
#define EEPROM_OFFSET_Z_ADDR 9
#define EEPROM_SCALE_X_ADDR 13
#define EEPROM_SCALE_Y_ADDR 17
#define EEPROM_SCALE_Z_ADDR 21


class Compass
{
private:
    uint16_t address;
    int16_t mx, my, mz;
    float offset_x, offset_y, offset_z;
    float scale_x, scale_y, scale_z;

public:

    Compass();
    void setup();
    bool read();

    // Getters
    inline float getMx() const { return (float)mx; } // dps
    inline float getMy() const { return (float)my; }
    inline float getMz() const { return (float)mz; }
};

#endif // !COMPASS_HPP