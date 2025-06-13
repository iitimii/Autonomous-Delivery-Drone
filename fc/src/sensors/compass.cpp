#include "sensors/compass.hpp"
#include "uav/i2c_utils.hpp"
#include "communication/debugging.hpp"
#include "uav/eeprom.hpp"

Compass::Compass() : address(0x1E), mx(0), my(0), mz(0)
{
}

void Compass::setup()
{
    debugging::log("Compass setup");
    i2c::check(address, 3);

    i2c::HWire.beginTransmission(address);
    i2c::HWire.write(0x00);       // We want to write to the Configuration Register A (00 hex).
    i2c::HWire.write(0x78);       // Set the Configuration Regiser A bits as 01111000 to set sample rate (average of 8 at 75Hz).
    i2c::HWire.write(0x20);       // Set the Configuration Regiser B bits as 00100000 to set the gain at +/-1.3Ga.
    i2c::HWire.write(0x00);       // Set the Mode Regiser bits as 00000000 to set Continues-Measurement Mode.
    i2c::HWire.endTransmission(); // End the transmission with the compass.

    byte signature = eeprom::read(EEPROM_SIGNATURE_ADDR, 0);
    if (signature != CALIBRATION_SIGNATURE)
    {
        debugging::log("Compass calibration data not found");
    }

    else
    {
        debugging::log("Compass calibration data found");
        
        offset_x = eeprom::read(EEPROM_OFFSET_X_ADDR, 0.0f);
        offset_y = eeprom::read(EEPROM_OFFSET_Y_ADDR, 0.0f);
        offset_z = eeprom::read(EEPROM_OFFSET_Z_ADDR, 0.0f);
        scale_x = eeprom::read(EEPROM_SCALE_X_ADDR, 1.0f);
        scale_y = eeprom::read(EEPROM_SCALE_Y_ADDR, 1.0f);
        scale_z = eeprom::read(EEPROM_SCALE_Z_ADDR, 1.0f);
    }
}

bool Compass::read()
{
    i2c::HWire.beginTransmission(address); // Start communication with the compass.
    i2c::HWire.write(0x03);                // We want to start reading at the hexadecimal location 0x03.
    if (i2c::HWire.endTransmission(false) != 0)
        return false;


    uint8_t buffer[6];
    if (i2c::HWire.requestFrom(address, (uint8_t)6, true) != 6)
        return false;

    i2c::HWire.readBytes(buffer, 6);

    mx = (buffer[0] << 8) | buffer[1];
    mz = (buffer[2] << 8) | buffer[3];
    my = (buffer[4] << 8) | buffer[5];

    mx = (mx - offset_x) * scale_x;
    my = (my - offset_y) * scale_y;
    mz = (mz - offset_z) * scale_z;

    return true;
}