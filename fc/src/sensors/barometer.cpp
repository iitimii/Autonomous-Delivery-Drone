#include "sensors/barometer.hpp"
#include "uav/i2c_utils.hpp"
#include "uav/led.hpp"
#include "communication/debugging.hpp"

Barometer::Barometer(uint16_t addr)
    : address(addr), pressure(0.0), initial_pressure(0.0), temperature(0.0)
{
}

void Barometer::read_calibration()
{    debugging::log("Barometer read_calibration");

    i2c::HWire.beginTransmission(address);
    i2c::HWire.write(0x88); // Starting register for calibration data
    i2c::HWire.endTransmission();

    i2c::HWire.requestFrom(address, 24);
    uint8_t buffer[24];
    i2c::HWire.readBytes(buffer, 24);
    dig_T1 = (buffer[0] | (buffer[1] << 8));
    dig_T2 = (buffer[2] | (buffer[3] << 8));
    dig_T3 = (buffer[4] | (buffer[5] << 8));
    dig_P1 = (buffer[6] | (buffer[7] << 8));
    dig_P2 = (buffer[8] | (buffer[9] << 8));
    dig_P3 = (buffer[10] | (buffer[11] << 8));
    dig_P4 = (buffer[12] | (buffer[13] << 8));
    dig_P5 = (buffer[14] | (buffer[15] << 8));
    dig_P6 = (buffer[16] | (buffer[17] << 8));
    dig_P7 = (buffer[18] | (buffer[19] << 8));
    dig_P8 = (buffer[20] | (buffer[21] << 8));
    dig_P9 = (buffer[22] | (buffer[23] << 8));
}

void Barometer::setup()
{
    debugging::log("Barometer setup");
    i2c::check(address, 2);

    // Configure the sensor - Example settings for xx oversampling
    i2c::HWire.beginTransmission(address);
    i2c::HWire.write(0xF4); // Control register
    i2c::HWire.write(0x57); // xx oversampling pressure and temperature -- EDIT THIS
    i2c::HWire.endTransmission();
    i2c::HWire.beginTransmission(address);
    i2c::HWire.write(0xF5);
    i2c::HWire.write(0x14);
    i2c::HWire.endTransmission();

    // Read calibration data
    read_calibration();
}

bool Barometer::read()
{
    i2c::HWire.beginTransmission(address);
    i2c::HWire.write(0xF7);
    if (i2c::HWire.endTransmission(false) != 0)
        return false;
        
    if (i2c::HWire.requestFrom(address, (uint8_t)6, false) != 6)
        return false;

    uint8_t buffer[6];
    i2c::HWire.readBytes(buffer, 6);
    uint32_t press_msb = buffer[0];
    uint32_t press_lsb = buffer[1];
    uint32_t press_xlsb = buffer[2];
    uint32_t temp_msb = buffer[3];
    uint32_t temp_lsb = buffer[4];
    uint32_t temp_xlsb = buffer[5];

    // uint32_t press_msb = i2c::HWire.read();
    // uint32_t press_lsb = i2c::HWire.read();
    // uint32_t press_xlsb = i2c::HWire.read();
    // uint32_t temp_msb = i2c::HWire.read();
    // uint32_t temp_lsb = i2c::HWire.read();
    // uint32_t temp_xlsb = i2c::HWire.read();
    

    uint32_t adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
    uint32_t adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

    // Temperature compensation formula
    signed long int var1, var2;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    signed long int t_fine = var1 + var2;
    temperature = (float)((t_fine * 5 + 128) >> 8);

    // Pressure compensation formula
    unsigned long int p;
    var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
    if (var1 == 0)
    {
        p = 0;
        return true;
    }
    p = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
    {
        p = (p << 1) / ((unsigned long int)var1);
    }
    else
    {
        p = (p / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((signed long int)(p >> 2)) * ((signed long int)dig_P8)) >> 13;
    p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4));

    pressure = (double)p / 100; // in hPa

    return true;
}

void Barometer::calibrate()
{

    int16_t iterations = 500;
    initial_pressure = 0;
    initial_altitude = 0;
    float altitude = 0.0f;
    double pressure_sum = 0;
    double altitude_sum = 0;

    for (int16_t i = 0; i < iterations; ++i)
    {
        debugging::log("Barometer calibration iteration: " + String(i));
        if (i % 50 == 0)
            led::blink_once();

        read();
        altitude = 44330 * (1 - pow(pressure / 1013.25, 0.1903)); // meters
        pressure_sum += pressure;
        altitude_sum += altitude;

        delay(4);
    }

    initial_pressure = pressure_sum / iterations;
    initial_altitude = altitude_sum / iterations;
}