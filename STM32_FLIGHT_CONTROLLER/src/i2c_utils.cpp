#include "i2c_utils.hpp"

TwoWire HWire(PB4, PA8);
uint8_t compass_address = 0x1E;
uint8_t baro_address = 0x76;

void i2c::setup()
{
    HWire.setClock(400000);
    HWire.begin();
    HWire.setClock(400000);
}

void i2c::scan() {
  uint8_t error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; ++address) {
    HWire.beginTransmission(address);
    error = HWire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Done\n");
  }

  delay(5000); 
}

void i2c::check(uint8_t address, uint8_t error_code)
{
  HWire.beginTransmission(address);
  error = HWire.endTransmission();
  while (error != 0)
  {
    error = error_code;
    send_telemetry();
    delay(4);
    HWire.beginTransmission(address);
    error = HWire.endTransmission();
  }
  delay(50);
}