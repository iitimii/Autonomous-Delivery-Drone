#ifndef IMU_HPP
#define IMU_HPP

#include <cstdint>


class IMU {
public:
    IMU(uint16_t address = DEFAULT_ADDRESS);
    void setup();
    void reset();
    bool read();
    void calibrate();
    void calibrateManual();

    // Getters
    inline float getGx() const { return gx / 65.5; } // dps
    inline float getGy() const { return gy / 65.5; }
    inline float getGz() const { return gz / 65.5; }
    inline float getAx() const { return ax * 9.81 / 4096.0; } // m/s^2
    inline float getAy() const { return ay * 9.81 / 4096.0; }
    inline float getAz() const { return (az + 4096) * 9.81 / 4096.0; }
    inline int16_t getTemperature() const { return temperature; }

private:
    static constexpr uint16_t DEFAULT_ADDRESS = 0x68;
    static constexpr uint8_t NBYTES = 14;

    uint16_t address;
    int16_t gx, gy, gz;
    int16_t gx_cal, gy_cal, gz_cal;
    int16_t ax, ay, az;
    int16_t ax_cal, ay_cal, az_cal;
    int16_t temperature;
    bool use_manual_calibration;
};

#endif // IMU_HPP