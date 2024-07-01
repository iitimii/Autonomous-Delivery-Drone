#include "gyro.hpp"
#include "i2c_utils.hpp"
#include "led.hpp"

namespace gyro
{
    constexpr uint16_t address = 0x68;
    constexpr float alpha = 0.995;
    constexpr float degrees_per_second = 0.0000610687;
    constexpr float dps_to_radians = 0.00000106585;

    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
    int16_t pitch_rate = 0, roll_rate = 0, yaw_rate = 0;
    float pitch_accelerometer = 0.0f, roll_accelerometer = 0.0f;

    int16_t acc_x = 0, acc_y = 0, acc_z = 0;
    int32_t acc_resultant = 0;
    int32_t roll_rate_cal = 0, pitch_rate_cal = 0, yaw_rate_cal = 0;
    double roll_rate_lpf = 0.0, pitch_rate_lpf = 0.0, yaw_rate_lpf = 0.0;
    int16_t temperature = 0;

    

    void setup()
    {
        i2c::check(address, 1);

        i2c::write(address, 0x6B, 0x00); // PWR_MGMT_1 register: activate the gyro.
        i2c::write(address, 0x1B, 0x08); // GYRO_CONFIG register: 500dps full scale.
        i2c::write(address, 0x1C, 0x10); // ACCEL_CONFIG register: +/- 8g full scale range.
        i2c::write(address, 0x1A, 0x03); // CONFIG register: Digital Low Pass Filter to ~43Hz.
    }

    void read()
    {
        const uint8_t nbytes = 14;
        i2c::HWire.beginTransmission(static_cast<uint8_t>(address));
        i2c::HWire.write(0x3B);
        i2c::HWire.endTransmission();
        i2c::HWire.requestFrom(static_cast<uint8_t>(address), nbytes);

        acc_y = (i2c::HWire.read() << 8) | i2c::HWire.read();
        acc_x = (i2c::HWire.read() << 8) | i2c::HWire.read();
        acc_z = (i2c::HWire.read() << 8) | i2c::HWire.read();
        temperature = (i2c::HWire.read() << 8) | i2c::HWire.read();
        roll_rate = (i2c::HWire.read() << 8) | i2c::HWire.read();
        pitch_rate = (i2c::HWire.read() << 8) | i2c::HWire.read();
        yaw_rate = (i2c::HWire.read() << 8) | i2c::HWire.read();

        acc_z *= -1;
        acc_x *= -1;

        roll_rate -= roll_rate_cal;
        pitch_rate -= pitch_rate_cal;
        yaw_rate -= yaw_rate_cal;
    }

    void calibrate()
    {
        roll_rate_cal = 0;
        pitch_rate_cal = 0;
        yaw_rate_cal = 0;

        for (uint16_t i = 0; i < 2000; ++i)
        {
            if (i % 125 == 0)
                led::blink_once(); // Change the LED status every 125 readings to indicate calibration.

            read();
            roll_rate_cal += roll_rate;
            pitch_rate_cal += pitch_rate;
            yaw_rate_cal += yaw_rate;

            delay(4); // Small delay to simulate a 250Hz loop during calibration.
        }

        roll_rate_cal /= 2000;
        pitch_rate_cal /= 2000;
        yaw_rate_cal /= 2000;
    }

    void get_acc_angle()
    {
        acc_resultant = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

        if (abs(acc_y) < acc_resultant)
            pitch_accelerometer = asin(static_cast<float>(acc_y) / acc_resultant) * 57.29578;

        if (abs(acc_x) < acc_resultant)
            roll_accelerometer = asin(static_cast<float>(acc_x) / acc_resultant) * 57.29578;
    }

    void filter()
    {
        constexpr float filter_coefficient = 0.3;
        constexpr float one_minus_filter_coefficient = 0.7;
        const double roll_rate_scaled = static_cast<double>(roll_rate) / 65.5;
        const double pitch_rate_scaled = static_cast<double>(pitch_rate) / 65.5;
        const double yaw_rate_scaled = static_cast<double>(yaw_rate) / 65.5;

        roll_rate_lpf = (roll_rate_lpf * one_minus_filter_coefficient) + (roll_rate_scaled * filter_coefficient);
        pitch_rate_lpf = (pitch_rate_lpf * one_minus_filter_coefficient) + (pitch_rate_scaled * filter_coefficient);
        yaw_rate_lpf = (yaw_rate_lpf * one_minus_filter_coefficient) + (yaw_rate_scaled * filter_coefficient);
    }

    void calculate_attitude()
    {
        filter();

        const double pitch_increment = static_cast<double>(pitch_rate) * degrees_per_second;
        const double roll_increment = static_cast<double>(roll_rate) * degrees_per_second;
        const double yaw_scaled = static_cast<double>(yaw_rate) * dps_to_radians;

        pitch += pitch_increment;
        roll += roll_increment;

        pitch -= roll * sin(yaw_scaled);
        roll += pitch * sin(yaw_scaled);

        get_acc_angle();

        pitch = pitch * alpha + pitch_accelerometer * (1 - alpha);
        roll = roll * alpha + roll_accelerometer * (1 - alpha);
    }
}
