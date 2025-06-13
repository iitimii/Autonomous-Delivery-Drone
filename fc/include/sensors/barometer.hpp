#ifndef BARO_HPP
#define BARO_HPP
#include <cstdint>

class Barometer
{
private:
    static constexpr uint16_t DEFAULT_ADDRESS = 0x76;
    uint16_t address;
    float pressure, initial_pressure, initial_altitude;
    float temperature;

    // Calibration data
    uint16_t dig_T1, dig_P1;
    int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
    int16_t dig_P6, dig_P7, dig_P8, dig_P9;

    void read_calibration();

public:
    Barometer(uint16_t address = DEFAULT_ADDRESS);
    float get_pressure() { return pressure; }
    float get_initial_pressure() { return initial_pressure; }
    float get_initial_altitude() { return initial_altitude; }
    float get_temperature() { return temperature; }
    void setup();
    bool read();
    void calibrate();
};
#endif // BARO_HPP