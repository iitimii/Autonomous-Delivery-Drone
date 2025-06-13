#include "sensors/imu.hpp"
#include "uav/i2c_utils.hpp"
#include "uav/led.hpp"
#include "communication/telemetry.hpp"
#include "communication/debugging.hpp"
#include <EEPROM.h>


IMU::IMU(uint16_t addr) : address(addr),
                          gy(0), gx(0), gz(0),
                          ax(0), ay(0), az(0),
                          gx_cal(-275), gy_cal(-119), gz_cal(-53),
                          ax_cal(285), ay_cal(96), az_cal(4501),
                          temperature(0), use_manual_calibration(false)
{
    // EEPROM.get(0, gx_cal);
    // EEPROM.get(2, gy_cal);
    // EEPROM.get(4, gz_cal);
    // EEPROM.get(6, ax_cal);
    // EEPROM.get(8, ay_cal);
}

void IMU::setup()
{
    delay(2);
    debugging::log("IMU setup");
    i2c::check(address, 1);

    i2c::write(address, 0x6B, 0x00); // PWR_MGMT_1 register: activate the gyro
    i2c::write(address, 0x1B, 0x08); // GYRO_CONFIG register: 500dps full scale
    i2c::write(address, 0x1C, 0x10); // ACCEL_CONFIG register: +/- 8g full scale range
    i2c::write(address, 0x1A, 0x03); // CONFIG register: Digital Low Pass Filter to ~43Hz
    delay(2);
}

void IMU::reset()
{
    // i2c::HWire.beginTransmission(static_cast<uint8_t>(address));
    // i2c::HWire.write(0x6B); // Power management register
    // i2c::HWire.write(0x80); // Set the reset bit
    // i2c::HWire.endTransmission();

    i2c::write(address, 0x6B, 0x00); // PWR_MGMT_1 register: activate the gyro
    i2c::write(address, 0x1B, 0x08); // GYRO_CONFIG register: 500dps full scale
    i2c::write(address, 0x1C, 0x10); // ACCEL_CONFIG register: +/- 8g full scale range
    i2c::write(address, 0x1A, 0x03); // CONFIG register: Digital Low Pass Filter to ~43Hz
}

bool IMU::read()
{
    delayMicroseconds(10);
    i2c::HWire.beginTransmission(address);
    i2c::HWire.write(0x3B);
    uint8_t err = i2c::HWire.endTransmission(true);
    if (err != 0) return false;

    int received = i2c::HWire.requestFrom(address, NBYTES, true);
    if (received != NBYTES) return false;

    if(i2c::HWire.available() < NBYTES) return false;

    uint8_t buffer[NBYTES];
    size_t bytesRead = i2c::HWire.readBytes(buffer, NBYTES);
    if (bytesRead != NBYTES) return false;

    ax = (buffer[0] << 8) | buffer[1];
    ay = (buffer[2] << 8) | buffer[3];
    az = ((buffer[4] << 8) | buffer[5]);
    temperature = (buffer[6] << 8) | buffer[7];
    gx = ((buffer[8] << 8) | buffer[9]);
    gy = ((buffer[10] << 8) | buffer[11]);
    gz = ((buffer[12] << 8) | buffer[13]);

    ay *= -1;
    az *= -1;
    gy *= -1;

    gx -= gx_cal;
    gy -= gy_cal;
    gz -= gz_cal;
    ax -= ax_cal;
    ay -= ay_cal;
    az -= az_cal;

    temperature = (temperature / 340) + 36.53;

    return true;
}

void IMU::calibrate()
{
    debugging::log("IMU calibration");
    if (use_manual_calibration)
        return;

    gx_cal = 0, gy_cal = 0, gz_cal = 0;
    int64_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t iterations = 1000;
    delay(1000);

    for (int16_t i = 0; i < iterations; ++i)
    {
        debugging::log("Iteration: " + String(i));
        if (i % 25 == 0)
            led::blink_once();

        read();

        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        delay(4);
    }

    gx_cal = gx_sum / iterations;
    gy_cal = gy_sum / iterations;
    gz_cal = gz_sum / iterations;
}

void IMU::calibrateManual()
{
    delay(2000);
    gx_cal = 0, gy_cal = 0, gz_cal = 0;
    ax_cal = 0, ay_cal = 0, az_cal = 0;
    int64_t gx_sum = 0, gy_sum = 0, gz_sum = 0, ax_sum = 0, ay_sum = 0, az_sum = 0;
    constexpr int32_t warmup = 1000;
    constexpr int32_t iterations = 20000;

    for (int32_t i = 0; i < warmup; ++i)
    {
        if (i % 125 == 0)
            led::blink_once();
        read();
        delay(2);
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
            az_sum += az;
        }
        delay(4);
    }
    led::on();

    gx_cal = gx_sum / iterations;
    gy_cal = gy_sum / iterations;
    gz_cal = gz_sum / iterations;
    ax_cal = ax_sum / iterations;
    ay_cal = ay_sum / iterations;
    az_cal = az_sum / iterations;

    while (1)
    {
        // telemetry::send();
        debugging::log("Calibration values:");
        debugging::log("gx_cal: " + String(gx_cal));
        debugging::log("gy_cal: " + String(gy_cal));
        debugging::log("gz_cal: " + String(gz_cal));
        debugging::log("ax_cal: " + String(ax_cal));
        debugging::log("ay_cal: " + String(ay_cal));
        debugging::log("az_cal: " + String(az_cal));
        delay(3000);
        delay(4);

        // Save calibration values to eeprom if message from serial is yes
        if (Serial.available() > 0)
        {
            String message = Serial.readString();
            if (message == "yes")
            {
                debugging::log("Saving calibration values to EEPROM");
                EEPROM.put(0, gx_cal);
                EEPROM.put(2, gy_cal);
                EEPROM.put(4, gz_cal);
                EEPROM.put(6, ax_cal);
                EEPROM.put(8, ay_cal);
                break;
            }
        }
    }
}