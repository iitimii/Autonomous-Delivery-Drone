/******************************************************************
  @file       readPressureUnits.ino
  @brief      Example sketch to return pressure in various units
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
  boards Rev. 1 and 2. 

  The available returned pressure units my be found in the Units
  enumeration. They are:

   - HECTOPASCAL,
   - PSI,
   - ATMOSPHERES,
   - MM_HG,
   - IN_HG,
   - MILLIBAR, and
   - KILOPASCAL.

  The default is HECTOPASCAL (hPa) which is the same as MILLIBAR.

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
    Serial.print("Pressure (hPa): ");  Serial.print(LPS22HB.readPressure());                   Serial.print(" hPa,    ");
    Serial.print("Pressure (PSI): ");  Serial.print(LPS22HB.readPressure(Units::PSI));         Serial.print(" PSI,    ");
    Serial.print("Pressure (kPa): ");  Serial.print(LPS22HB.readPressure(Units::KILOPASCAL));  Serial.println(" kPa.");

    Serial.print("Pressure (atm): ");  Serial.print(LPS22HB.readPressure(Units::ATMOSPHERES)); Serial.print(" atm,    ");
    Serial.print("Pressure (mmHg): "); Serial.print(LPS22HB.readPressure(Units::MM_HG));       Serial.print(" mmHg,    ");
    Serial.print("Pressure (inHg): "); Serial.print(LPS22HB.readPressure(Units::IN_HG));       Serial.println(" inHg.\n");
    delay(1000);
}