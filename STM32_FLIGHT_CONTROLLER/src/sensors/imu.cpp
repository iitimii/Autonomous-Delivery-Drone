#include "sensors/imu.hpp"
#include "uav/i2c_utils.hpp"
#include "uav/led.hpp"
#include "communication/telemetry.hpp"

IMU::IMU(uint16_t addr) : address(addr),
                          gy(0), gx(0), gz(0),
                          ax(0), ay(0), az(0),
                          gx_cal(-268), gy_cal(124), gz_cal(-47),
                          ax_cal(257), ay_cal(92),
                          temperature(0), use_manual_calibration(false) {}

void IMU::setup()
{
    i2c::check(address, 1);

    i2c::write(address, 0x6B, 0x00); // PWR_MGMT_1 register: activate the gyro
    i2c::write(address, 0x1B, 0x08); // GYRO_CONFIG register: 500dps full scale
    i2c::write(address, 0x1C, 0x10); // ACCEL_CONFIG register: +/- 8g full scale range
    i2c::write(address, 0x1A, 0x03); // CONFIG register: Digital Low Pass Filter to ~43Hz
}

void IMU::reset()
{
    i2c::HWire.beginTransmission(static_cast<uint8_t>(address));
    i2c::HWire.write(0x6B); // Power management register
    i2c::HWire.write(0x80); // Set the reset bit
    i2c::HWire.endTransmission();

    delayMicroseconds(50);

    i2c::write(address, 0x6B, 0x00); // PWR_MGMT_1 register: activate the gyro
    i2c::write(address, 0x1B, 0x08); // GYRO_CONFIG register: 500dps full scale
    i2c::write(address, 0x1C, 0x10); // ACCEL_CONFIG register: +/- 8g full scale range
    i2c::write(address, 0x1A, 0x03); // CONFIG register: Digital Low Pass Filter to ~43Hz
}

bool IMU::read()
{
    static uint8_t failureCount = 0;
    i2c::HWire.beginTransmission(static_cast<uint8_t>(address));
    i2c::HWire.write(0x3B);
    i2c::HWire.endTransmission();

    if (i2c::HWire.requestFrom(static_cast<uint8_t>(address), NBYTES) != NBYTES)
    {
        if (++failureCount >= 100)
        {
            reset();
            failureCount = 0;
        }
        return false;
    }

    failureCount = 0;

    uint8_t buffer[NBYTES];
    i2c::HWire.readBytes(buffer, NBYTES);

    ax = (buffer[0] << 8) | buffer[1];
    ay = (buffer[2] << 8) | buffer[3];
    az = ((buffer[4] << 8) | buffer[5]);
    temperature = (buffer[6] << 8) | buffer[7];
    gx = ((buffer[8] << 8) | buffer[9]);
    gy = ((buffer[10] << 8) | buffer[11]);
    gz = ((buffer[12] << 8) | buffer[13]);

    az *= -1;
    ay *= -1;
    gy *= -1;
    gz *= -1;

    gx -= gx_cal;
    gy -= gy_cal;
    gz -= gz_cal;
    ax -= ax_cal;
    ay -= ay_cal;

    temperature = (temperature / 340) + 36.53;

    return true;
}

void IMU::calibrate()
{
    if (use_manual_calibration)
        return;

    gx_cal = 0, gy_cal = 0, gz_cal = 0;
    int64_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t iterations = 2000;

    for (int16_t i = 0; i < iterations; ++i)
    {
        if (i % 25 == 0)
            led::blink_once();

        read();

        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        delay(1);
    }

    gx_cal = gx_sum / iterations;
    gy_cal = gy_sum / iterations;
    gz_cal = gz_sum / iterations;
}

void IMU::calibrateManual()
{
    delay(2000);

    int64_t gx_sum = 0, gy_sum = 0, gz_sum = 0, ax_sum = 0, ay_sum = 0;
    constexpr int32_t warmup = 1000;
    constexpr int32_t iterations = 10000;

    for (int32_t i = 0; i < warmup; ++i)
    {
        if (i % 125 == 0)
            led::blink_once();
        read();
        delay(4);
    }

    for (int32_t i = 0; i < iterations; ++i)
    {
        if (i % 25 == 0)
            led::blink_once();

        if (read())
        {
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
            ax_sum += ax;
            ay_sum += ay;
        }
        delay(4);
    }
    led::on();

    gx_cal = gx_sum / iterations;
    gy_cal = gy_sum / iterations;
    gz_cal = gz_sum / iterations;
    ax_cal = ax_sum / iterations;
    ay_cal = ay_sum / iterations;

    while (1)
    {
        telemetry::send();
        delay(4);
    }
}