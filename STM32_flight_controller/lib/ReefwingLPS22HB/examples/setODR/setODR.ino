/******************************************************************
  @file       setODR.ino
  @brief      Example sketch to continuously sample pressure
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

  The sensor can be read in one-shot mode (i.e., on demand) or
  continuously sampled. The available Output Data Rates (ODR)
  may be found in the Rates enum. They are:

   - RATE_ONE_SHOT
   - RATE_1_HZ
   - RATE_10_HZ
   - RATE_25_HZ
   - RATE_50_HZ
   - RATE_75_HZ

  The default rate for the library is one-shot.
  
  You can adjust the loop() delay value and ODR rates to see how this
  impacts sensor readings.

******************************************************************/

#include <ReefwingLPS22HB.h>

ReefwingLPS22HB LPS22HB;

unsigned long loopCounter = 0, elapsedTime = 0;

void setup() {
    LPS22HB.begin();

    //  Start Serial and wait for connection
    Serial.begin(115200);
    while (!Serial);

    if (LPS22HB.connected()) {
        Serial.println("LPS22HB Pressure Sensor found.");
        LPS22HB.setODR(Rate::RATE_10_HZ);
    }
    else {
        Serial.println("LPS22HB Pressure Sensor not found.");
        while (true) {  }   // loop forever
    }
}

void loop() {
    //  Loop Speed
    long currentTime = millis();
    loopCounter++;

    // Sensor Reading
    Serial.print("Pressure: "); Serial.print(LPS22HB.readPressure()); Serial.print(" hPa, ");
    Serial.print("Temperature: "); Serial.print(LPS22HB.readTemperature()); Serial.print(" C, ");
    Serial.print("Altitude: "); Serial.print(LPS22HB.readAltitude()); Serial.println(" m");
    delay(100);

    if (currentTime - elapsedTime > 1000){
        Serial.print("Loop Frequency: "); Serial.print(loopCounter); Serial.println(" Hz");
    
        elapsedTime = currentTime;
        loopCounter = 0;
    }
}
