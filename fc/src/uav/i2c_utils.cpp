#include "uav/i2c_utils.hpp"
#include "communication/debugging.hpp"
#include "communication/telemetry.hpp"
#include "uav/drone.hpp"
#include "uav/led.hpp"
#include "driver/i2c.h"
#include "esp_log.h"


TwoWire i2c::HWire(0);

void i2c::setup()
{
  HWire.setPins(21, 22); // SDA, SCL
  // HWire.setTimeOut(1);
  // HWire.setTimeout(1);
  HWire.begin(21, 22, 400000);
  // i2c_set_timeout(I2C_NUM_0, 0); // Timeout in APB clock cycles (e.g., 10,000 cycles is ~100 microseconds)  
  // i2c_set_timeout(I2C_NUM_1, 0); // Timeout in APB clock cycles (e.g., 10,000 cycles is ~100 microseconds)

}


void i2c::check(const uint8_t &address, const uint8_t &error_code)
{
  HWire.beginTransmission(address);
  drone::error = HWire.endTransmission();
  while (drone::error != 0)
  {
    drone::error = error_code;
    // telemetry::send();
    String message = "I2C error at address: " + String(address) + " Code: " + String(drone::error);
    debugging::log(message);
    delay(3);
    HWire.beginTransmission(address);
    drone::error = HWire.endTransmission();
  }
}

void i2c::write(uint8_t address, uint8_t reg, uint8_t value)
{
  HWire.beginTransmission(address);
  HWire.write(reg);
  HWire.write(value);
  HWire.endTransmission();
}

void i2c::scan() {
  uint8_t error, address;
  int nDevices;

  debugging::log("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; ++address) {
    HWire.beginTransmission(address);
    error = HWire.endTransmission();

    if (error == 0) {
      debugging::log("I2C device found at address 0x");
      if (address < 16) {
        debugging::log("0");
      }
      debugging::print_address(address);
      debugging::log("  !");
      nDevices++;
    } else if (error == 4) {
      debugging::log("Unknown error at address 0x");
      if (address < 16) {
        debugging::log("0");
      }
      debugging::print_address(address);
    }
  }

  if (nDevices == 0) {
    debugging::log("No I2C devices found\n");
  } else {
    debugging::log("Done\n");
  }

  delay(5000);
}