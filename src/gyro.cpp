// #include <Arduino.h>
// #include <Wire.h>

// #define PSCL PB10
// #define PSDA PB3
       

// int16_t Gyro_X, Gyro_Y, Gyro_Z;

// TwoWire HWire (PSDA, PSCL); 

// void setup() {
//     Serial.begin(57600);
//     HWire.setClock(400000); // I2C fast mode, 400kHz other I2C run on 100kHz
//     HWire.begin();  
//     delay(250);

//     HWire.beginTransmission(0x68); // Start communicating with the MPU-6050 I2C bus
//     HWire.write(0x6B); // Send the requested starting register
//     HWire.write(0x00); // Set the power management register to 0 (wake up MPU-6050)
//     HWire.endTransmission(); // End the transmission
// }

// void loop(){
//     HWire.beginTransmission(0x68);
//     HWire.write(0x43);
//     HWire.endTransmission();
//     HWire.requestFrom(0x68,6);
//     // Now  6 bytes of data are available in the memory 
//     Gyro_X = HWire.read() << 8 | HWire.read(); 
//     Gyro_Y = HWire.read() << 8 | HWire.read();
//     Gyro_Z = HWire.read() << 8 | HWire.read();
//     Serial.print("x = ");
//     Serial.print(Gyro_X);
//     Serial.print(" y = ");
//     Serial.print(Gyro_Y);
//     Serial.print(" z = ");
//     Serial.println(Gyro_Z);
//     delay(250);
// }