void scan_I2C() {
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

void check_device(uint8_t address, uint8_t error_code)
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