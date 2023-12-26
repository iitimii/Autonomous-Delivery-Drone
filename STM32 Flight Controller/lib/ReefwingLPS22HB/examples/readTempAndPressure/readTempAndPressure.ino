/******************************************************************
  @file       readTempAndPressure.ino
  @brief      Example sketch to read temperature & pressure
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.3
  Date:        21/02/23

  1.0.1 Original Release.                         11/02/23
  1.0.2 Added 2's comp for temperature            20/02/23
  1.0.3 Fixed setODR() bug                        21/02/23

  Acquires temperature and pressure readings from the LPS22HB
  Pressure Sensor mounted on the Arduino Nano 33 BLE Sense
  boards Rev. 1 and 2. Altitude is calculated using the barometric
  equation and is relative to the reference pressure passed into
  the method.

  This example uses the defaults for readPressure(), readTemperature(),
  and readAltitude(). These are respectively, pressure Units = hPa/mbar, 
  temperature Scales = Celcius and Pressure Reference = QNE = 1013.25 hPa = 1 atm. 
  
  See README for additional detail.

******************************************************************/

#include <ReefwingLPS22HB.h>

ReefwingLPS22HB LPS22HB;

void setup() {
    LPS22HB.begin();

    //  Start Serial and wait for connection
    Serial.begin(115200);
    while (!Serial);

    if (LPS22HB.connected()) {
        Serial.println("LPS22HB Pressure Sensor found.");
    }
    else {
        Serial.println("LPS22HB Pressure Sensor not found.");
        while (true) {  }   // loop forever
    }
}

void loop() {
    // Sensor Reading Loop
    Serial.print("Pressure: "); Serial.print(LPS22HB.readPressure()); Serial.print(" hPa, ");
    Serial.print("Temperature: "); Serial.print(LPS22HB.readTemperature()); Serial.print(" C, ");
    Serial.print("Altitude: "); Serial.print(LPS22HB.readAltitude()); Serial.println(" m");
    delay(1000);
}