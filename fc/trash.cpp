// int16_t my = i2c::HWire.read() << 8 | i2c::HWire.read();
    // int16_t mz = i2c::HWire.read() << 8 | i2c::HWire.read();
    // int16_t mx = i2c::HWire.read() << 8 | i2c::HWire.read();

    // mx *= -1;
    // my *= -1;


    // Before the compass can give accurate measurements it needs to be calibrated. At startup the compass_offset and compass_scale
    // variables are calculated. The following part will adjust the raw compas values so they can be used for the calculation of the heading.
    // if (compass_calibration_on == 0)
    // {                                  // When the compass is not beeing calibrated.
    //     compass_y += compass_offset_y; // Add the y-offset to the raw value.
    //     compass_y *= compass_scale_y;  // Scale the y-value so it matches the other axis.
    //     compass_z += compass_offset_z; // Add the z-offset to the raw value.
    //     compass_z *= compass_scale_z;  // Scale the z-value so it matches the other axis.
    //     compass_x += compass_offset_x; // Add the x-offset to the raw value.
    // }

    // roll *= DEG_TO_RAD;
    // pitch *= -DEG_TO_RAD;
    
    // float mx_horizontal = (float)mx * cos(pitch) + (float)my * sin(roll) * sin(pitch) - (float)mz * cos(roll) * sin(pitch);
    // float my_horizontal = (float)my * cos(roll) + (float)mz * sin(roll);

    // if (my_horizontal < 0)
    //     heading = 180 + (180 + ((atan2(my_horizontal, mx_horizontal))));
    // else
    //     heading = (atan2(my_horizontal, mx_horizontal));

    // heading *= RAD_TO_DEG;
    // heading += declination; // Add the declination to the magnetic compass heading to get the geographic north.

    // if (heading < 0)
    //     heading += 360; // If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
    // else if (heading >= 360)
    //     heading -= 360; // If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.


    // Calibration

//     #include <Arduino.h>
// #include <Wire.h>
// #include <EEPROM.h>

// // HMC5883L Register Addresses
// #define HMC5883L_ADDR 0x1E
// #define CONFIG_REG_A 0x00
// #define CONFIG_REG_B 0x01
// #define MODE_REG 0x02
// #define DATA_X_MSB 0x03

// // EEPROM Addresses for storing calibration data
// #define EEPROM_SIZE 24
// #define CALIBRATION_SIGNATURE 0xAB // Signature to verify calibration data is valid
// #define EEPROM_SIGNATURE_ADDR 0
// #define EEPROM_OFFSET_X_ADDR 1
// #define EEPROM_OFFSET_Y_ADDR 5
// #define EEPROM_OFFSET_Z_ADDR 9
// #define EEPROM_SCALE_X_ADDR 13
// #define EEPROM_SCALE_Y_ADDR 17
// #define EEPROM_SCALE_Z_ADDR 21

// // LED indicating calibration status
// #define LED_PIN 2

// // Global variables
// int16_t magX, magY, magZ;
// int16_t magXmin, magYmin, magZmin;
// int16_t magXmax, magYmax, magZmax;
// float offsetX, offsetY, offsetZ;
// float scaleX, scaleY, scaleZ;
// bool calibrationStarted = false;
// unsigned long calibrationStartTime = 0;
// const unsigned long calibrationDuration = 30000; // 30 seconds for calibration

// void setupHMC5883L() {
//   Wire.beginTransmission(HMC5883L_ADDR);
//   Wire.write(CONFIG_REG_A);  // Set to 8 samples @ 15Hz
//   Wire.write(0x70);
//   Wire.endTransmission();

//   Wire.beginTransmission(HMC5883L_ADDR);
//   Wire.write(CONFIG_REG_B);  // 1.3 gain LSb / Gauss 1090 (default)
//   Wire.write(0x20);
//   Wire.endTransmission();

//   Wire.beginTransmission(HMC5883L_ADDR);
//   Wire.write(MODE_REG);      // Continuous measurement mode
//   Wire.write(0x00);
//   Wire.endTransmission();
// }

// bool readMagnetometer() {
//   Wire.beginTransmission(HMC5883L_ADDR);
//   Wire.write(DATA_X_MSB);  // Start reading from X MSB register
//   Wire.endTransmission();
  
//   Wire.requestFrom(HMC5883L_ADDR, 6);
//   if (Wire.available() >= 6) {
//     magX = Wire.read() << 8 | Wire.read();
//     magZ = Wire.read() << 8 | Wire.read(); // Note: for HMC5883L, Z is the second register pair
//     magY = Wire.read() << 8 | Wire.read();
//     return true;
//   }
//   return false;
// }

// void startCalibration() {
//   Serial.println("Starting magnetometer calibration...");
//   Serial.println("Please rotate the sensor in all directions for 30 seconds.");
  
//   // Initialize min/max values
//   magXmin = magYmin = magZmin = 32767;
//   magXmax = magYmax = magZmax = -32768;
  
//   calibrationStarted = true;
//   calibrationStartTime = millis();
// }

// void updateCalibration() {
//   if (!calibrationStarted) return;
  
//   // Read magnetometer data
//   if (readMagnetometer()) {
//     // Update min/max values
//     magXmin = min(magXmin, magX);
//     magYmin = min(magYmin, magY);
//     magZmin = min(magZmin, magZ);
    
//     magXmax = max(magXmax, magX);
//     magYmax = max(magYmax, magY);
//     magZmax = max(magZmax, magZ);
//   }
  
//   // Blink LED to indicate calibration in progress
//   digitalWrite(LED_PIN, (millis() / 500) % 2);
  
//   // Check if calibration time has elapsed
//   if (millis() - calibrationStartTime > calibrationDuration) {
//     finishCalibration();
//   }
// }

// void finishCalibration() {
//   calibrationStarted = false;
//   digitalWrite(LED_PIN, HIGH); // Solid LED to indicate calibration complete
  
//   // Calculate offsets and scaling factors
//   offsetX = (magXmax + magXmin) / 2.0;
//   offsetY = (magYmax + magYmin) / 2.0;
//   offsetZ = (magZmax + magZmin) / 2.0;
  
//   // Calculate average delta
//   float deltaX = (magXmax - magXmin) / 2.0;
//   float deltaY = (magYmax - magYmin) / 2.0;
//   float deltaZ = (magZmax - magZmin) / 2.0;
//   float avgDelta = (deltaX + deltaY + deltaZ) / 3.0;
  
//   // Calculate scale factors
//   scaleX = avgDelta / deltaX;
//   scaleY = avgDelta / deltaY;
//   scaleZ = avgDelta / deltaZ;
  
//   // Print calibration results
//   Serial.println("Calibration complete!");
//   Serial.println("Min values: X=" + String(magXmin) + ", Y=" + String(magYmin) + ", Z=" + String(magZmin));
//   Serial.println("Max values: X=" + String(magXmax) + ", Y=" + String(magYmax) + ", Z=" + String(magZmax));
//   Serial.println("Offsets: X=" + String(offsetX) + ", Y=" + String(offsetY) + ", Z=" + String(offsetZ));
//   Serial.println("Scale factors: X=" + String(scaleX) + ", Y=" + String(scaleY) + ", Z=" + String(scaleZ));
  
//   // Save calibration to EEPROM
//   saveCalibrationToEEPROM();
// }

// void saveCalibrationToEEPROM() {
//   EEPROM.write(EEPROM_SIGNATURE_ADDR, CALIBRATION_SIGNATURE);
  
//   EEPROM.put(EEPROM_OFFSET_X_ADDR, offsetX);
//   EEPROM.put(EEPROM_OFFSET_Y_ADDR, offsetY);
//   EEPROM.put(EEPROM_OFFSET_Z_ADDR, offsetZ);
  
//   EEPROM.put(EEPROM_SCALE_X_ADDR, scaleX);
//   EEPROM.put(EEPROM_SCALE_Y_ADDR, scaleY);
//   EEPROM.put(EEPROM_SCALE_Z_ADDR, scaleZ);
  
//   EEPROM.commit();
//   Serial.println("Calibration data saved to EEPROM");
// }

// bool loadCalibrationFromEEPROM() {
//   // Check if valid calibration data exists
//   byte signature = EEPROM.read(EEPROM_SIGNATURE_ADDR);
//   if (signature != CALIBRATION_SIGNATURE) {
//     Serial.println("No valid calibration data found in EEPROM");
//     return false;
//   }
  
//   // Load calibration values
//   EEPROM.get(EEPROM_OFFSET_X_ADDR, offsetX);
//   EEPROM.get(EEPROM_OFFSET_Y_ADDR, offsetY);
//   EEPROM.get(EEPROM_OFFSET_Z_ADDR, offsetZ);
  
//   EEPROM.get(EEPROM_SCALE_X_ADDR, scaleX);
//   EEPROM.get(EEPROM_SCALE_Y_ADDR, scaleY);
//   EEPROM.get(EEPROM_SCALE_Z_ADDR, scaleZ);
  
//   Serial.println("Loaded calibration data from EEPROM");
//   Serial.println("Offsets: X=" + String(offsetX) + ", Y=" + String(offsetY) + ", Z=" + String(offsetZ));
//   Serial.println("Scale factors: X=" + String(scaleX) + ", Y=" + String(scaleY) + ", Z=" + String(scaleZ));
  
//   return true;
// }

// float getCalibratedHeading() {
//   if (!readMagnetometer()) {
//     return -1; // Error reading magnetometer
//   }
  
//   // Apply calibration
//   float calibX = (magX - offsetX) * scaleX;
//   float calibY = (magY - offsetY) * scaleY;
//   float calibZ = (magZ - offsetZ) * scaleZ;
  
//   // Calculate heading
//   float heading = atan2(calibY, calibX);
  
//   // Convert radians to degrees
//   heading = heading * 180.0 / PI;
  
//   // Normalize to 0-360
//   if (heading < 0) {
//     heading += 360;
//   }
  
//   return heading;
// }

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();
//   pinMode(LED_PIN, OUTPUT);
  
//   // Initialize EEPROM with specified size
//   EEPROM.begin(EEPROM_SIZE);
  
//   setupHMC5883L();
  
//   // Check if calibration data exists and load it
//   if (!loadCalibrationFromEEPROM()) {
//     Serial.println("Press 'c' to begin calibration");
//   } else {
//     digitalWrite(LED_PIN, HIGH); // Indicate calibration is loaded
//   }
// }

// void loop() {
//   if (calibrationStarted) {
//     updateCalibration();
//   } else {
//     // Check for command input
//     if (Serial.available() > 0) {
//       char cmd = Serial.read();
//       if (cmd == 'c' || cmd == 'C') {
//         startCalibration();
//       } else if (cmd == 'r' || cmd == 'R') {
//         float heading = getCalibratedHeading();
//         Serial.println("Current heading: " + String(heading) + " degrees");
//       }
//     }
    
//     // Read raw magnetometer values occasionally when not calibrating
//     if (!calibrationStarted && (millis() % 1000) < 10) {
//       readMagnetometer();
//       Serial.println("Raw magnetometer values - X: " + String(magX) + ", Y: " + String(magY) + ", Z: " + String(magZ));
//     }
//   }
  
//   delay(10);
// }