#include "gyro.hpp"
#include "i2c_utils.hpp"
#include "led.hpp"
#include "telemetry.hpp"

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
    float roll_rate_lpf = 0.0, pitch_rate_lpf = 0.0, yaw_rate_lpf = 0.0;
    int16_t temperature = 0;

    bool use_manual_calibration = true;
    int32_t roll_rate_cal = -268, pitch_rate_cal = 124, yaw_rate_cal = -47;
    int32_t roll_accelerometer_cal = 257, pitch_accelerometer_cal = 92;

    void setup()
    {
        i2c::check(address, 1);

        i2c::write(address, 0x6B, 0x00); // PWR_MGMT_1 register: activate the gyro.
        i2c::write(address, 0x1B, 0x08); // GYRO_CONFIG register: 500dps full scale.
        i2c::write(address, 0x1C, 0x10); // ACCEL_CONFIG register: +/- 8g full scale range.
        i2c::write(address, 0x1A, 0x03); // CONFIG register: Digital Low Pass Filter to ~43Hz.
    }

    void reset()
    {
        // Power down the MPU6050
        i2c::HWire.beginTransmission(static_cast<uint8_t>(address));
        i2c::HWire.write(0x6B); // Power management register
        i2c::HWire.write(0x80); // Set the reset bit
        i2c::HWire.endTransmission();

        delayMicroseconds(50);

        setup();
    }

    void read()
    {
        static uint8_t failureCount = 0;
        const uint8_t nbytes = 14;
        i2c::HWire.beginTransmission(static_cast<uint8_t>(address));
        i2c::HWire.write(0x3B);
        i2c::HWire.endTransmission();
        uint8_t bytesRead = i2c::HWire.requestFrom(static_cast<uint8_t>(address), nbytes);

        if (bytesRead != nbytes)
        {
            ++failureCount;
            if (failureCount >= 5)
            {
                reset();
                failureCount = 0;
            }
            return;
        }

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

        acc_y -= pitch_accelerometer_cal;
        acc_x -= roll_accelerometer_cal;
    }

    void calibrate()
    {
        if (!use_manual_calibration)
        {
            roll_rate_cal = 0;
            pitch_rate_cal = 0;
            yaw_rate_cal = 0;

            for (uint16_t i = 1; i <= 4000; ++i)
            {
                if (i % 25 == 0)
                    led::blink_once();

                read();
                // roll_rate_cal += roll_rate;
                // pitch_rate_cal += pitch_rate;
                // yaw_rate_cal += yaw_rate;

                roll_rate_cal = ((i - 1) / i) * roll_rate_cal + (roll_rate / i);
                pitch_rate_cal = ((i - 1) / i) * pitch_rate_cal + (pitch_rate / i);
                yaw_rate_cal = ((i - 1) / i) * yaw_rate_cal + (yaw_rate / i);

                delay(4);
            }
            // roll_rate_cal /= 4000;
            // pitch_rate_cal /= 4000;
            // yaw_rate_cal /= 4000;
        }
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
        const float roll_rate_scaled = static_cast<float>(roll_rate) / 65.5;
        const float pitch_rate_scaled = static_cast<float>(pitch_rate) / 65.5;
        const float yaw_rate_scaled = static_cast<float>(yaw_rate) / 65.5;

        roll_rate_lpf = (roll_rate_lpf * one_minus_filter_coefficient) + (roll_rate_scaled * filter_coefficient);
        pitch_rate_lpf = (pitch_rate_lpf * one_minus_filter_coefficient) + (pitch_rate_scaled * filter_coefficient);
        yaw_rate_lpf = (yaw_rate_lpf * one_minus_filter_coefficient) + (yaw_rate_scaled * filter_coefficient);
    }

    void calculate_attitude()
    {
        filter();

        const float pitch_increment = static_cast<float>(pitch_rate) * degrees_per_second;
        const float roll_increment = static_cast<float>(roll_rate) * degrees_per_second;
        const float yaw_scaled = static_cast<float>(yaw_rate) * dps_to_radians;

        pitch += pitch_increment;
        roll += roll_increment;

        pitch -= roll * sin(yaw_scaled);
        roll += pitch * sin(yaw_scaled);

        get_acc_angle();

        pitch = pitch * alpha + pitch_accelerometer * (1 - alpha);
        roll = roll * alpha + roll_accelerometer * (1 - alpha);
    }

    void calibrate_manual()
    {
        delay(2000);

        roll_rate_cal = 0;
        pitch_rate_cal = 0;
        yaw_rate_cal = 0;
        roll_accelerometer_cal = 0;
        pitch_accelerometer_cal = 0;

        for (uint16_t i = 0; i < 1000; ++i)
        {
            if (i % 125 == 0)
                led::blink_once();

            read();
            delay(4);
        }

        for (uint16_t i = 0; i < 10000; ++i)
        {
            if (i % 25 == 0)
                led::blink_once();

            read();
            roll_rate_cal += roll_rate;
            pitch_rate_cal += pitch_rate;
            yaw_rate_cal += yaw_rate;

            pitch_accelerometer_cal += acc_y;
            roll_accelerometer_cal += acc_x;

            delay(4);
        }
        led::on();

        roll_rate_cal /= 10000;
        pitch_rate_cal /= 10000;
        yaw_rate_cal /= 10000;

        pitch_accelerometer_cal /= 10000;
        roll_accelerometer_cal /= 10000;

        roll = roll_rate_cal;
        pitch = pitch_rate_cal;
        yaw = yaw_rate_cal;

        roll_accelerometer = roll_accelerometer_cal;
        pitch_accelerometer = pitch_accelerometer_cal;

        while (1)
        {
            telemetry::send();
            delay(4);
        }
    }
}
