/******************************************************************
  @file       readAltitude.ino
  @brief      Example sketch to read altitude
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

  For the QNH altitude you will need to obtain the latest value,
  and insert it in the setQNH() method.
  In Australia, QNH can be found on the BOM website:
  http://www.bom.gov.au/aviation/forecasts/area-qnh/

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
        LPS22HB.setQNH(1017.0f);      //  INSERT CURRENT QNH FOR YOUR AREA
    }
    else {
        Serial.println("LPS22HB Pressure Sensor not found.");
        while (true) {  }   // loop forever
    }
}

void loop() {
    // Sensor Reading Loop
    Serial.print("QNE Altitude: "); Serial.print(LPS22HB.readAltitude()); Serial.print(" m,    ");
    Serial.print("QFE Altitude: "); Serial.print(LPS22HB.readAltitude(PressureReference::QFE)); Serial.print(" m,    ");
    Serial.print("QNH Altitude: "); Serial.print(LPS22HB.readAltitude(PressureReference::QNH)); Serial.println(" m.");
    delay(1000);
}