#include "i2c_utils.hpp"
#include "telemetry.hpp"
#include "drone.hpp"

TwoWire i2c::HWire(PB4, PA8);

void i2c::setup()
{
    HWire.setClock(400000);
    HWire.begin();
    HWire.setClock(400000);
}


void i2c::check(const uint8_t& address, const uint8_t& error_code)
{
  HWire.beginTransmission(address);
   drone::error = HWire.endTransmission();
  while (drone::error != 0)
  {
    drone::error = error_code;
    telemetry::send();
    delay(4);
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

// void i2c::scan() {
//   uint8_t error, address;
//   int nDevices;

//   Serial.println("Scanning...");

//   nDevices = 0;
//   for (address = 1; address < 127; ++address) {
//     HWire.beginTransmission(address);
//     error = HWire.endTransmission();

//     if (error == 0) {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16) {
//         Serial.print("0");
//       }
//       Serial.print(address, HEX);
//       Serial.println("  !");
//       nDevices++;
//     } else if (error == 4) {
//       Serial.print("Unknown error at address 0x");
//       if (address < 16) {
//         Serial.print("0");
//       }
//       Serial.println(address, HEX);
//     }
//   }

//   if (nDevices == 0) {
//     Serial.println("No I2C devices found\n");
//   } else {
//     Serial.println("Done\n");
//   }

//   delay(5000); 
// }