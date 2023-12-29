// #define LPS22HB_WHO_AM_I        0X0F // Who am I
// #define LPS22HB_RES_CONF        0X1A // Normal (0) or Low current mode (1)
// #define LPS22HB_CTRL_REG1       0X10 // Output rate and filter settings
// #define LPS22HB_CTRL_REG2       0X11 // BOOT FIFO_EN STOP_ON_FTH IF_ADD_INC I2C_DIS SWRESET One_Shot
// #define LPS22HB_STATUS_REG      0X27 // Temp or Press data available bits
// #define LPS22HB_PRES_OUT_XL     0X28 // XLSB
// #define LPS22HB_PRES_OUT_L      0X29 // LSB
// #define LPS22HB_PRES_OUT_H      0X2A // MSB
// #define LPS22HB_TEMP_OUT_L      0X2B // LSB
// #define LPS22HB_TEMP_OUT_H      0X2C // MSB
// #define LPS22HB_WHO_AM_I_VALUE  0xB1 // Expected return value of WHO_AM_I register
// #define FIFO_CTRL 0x14 // FIFO


// #define LPS22HB_ADDRESS   0x5D  //  Address of barometer 


// #define STANDARD_ATMOSPHERE 1013.25   //  Average Sea-Level Pressure
// #define ALTITUDE_EXPONENT   0.190266  //  exp = -(R * Lr) / gM
// #define TEMP_LAPSE_RATE     0.0065    //  Lr = -0.0065 [K/m]
// #define HPA_TO_MMHG         0.750062  //  Convert hPa to mmHg
// #define HPA_TO_INHG         0.029530  //  Convert hPa to inHg


// uint8_t LPSread(uint8_t reg) {
//   HWire.beginTransmission(LPS22HB_ADDRESS);
//   HWire.write(reg);
//   HWire.endTransmission();
//   HWire.requestFrom(LPS22HB_ADDRESS, 1);
//   return HWire.read();
// }

// void LPSwrite(uint8_t reg, uint8_t data) {
//   HWire.beginTransmission(LPS22HB_ADDRESS);
//   HWire.write(reg);
//   HWire.write(data);
//   HWire.endTransmission();
// }



// void baro_setup(){
// while((LPSread(LPS22HB_CTRL_REG2) & 0x07) != 0) { //TODO change this code
//     yield();
//   }
// 	LPSwrite(LPS22HB_CTRL_REG1, 0b01001110); // 50Hz
// }


// void read_baro(){
//     P = (LPSread(LPS22HB_PRES_OUT_XL) |
//             (LPSread(LPS22HB_PRES_OUT_L) << 8) |
//             (LPSread(LPS22HB_PRES_OUT_H) << 16)) / 40.96;

    
//     T = (LPSread(LPS22HB_TEMP_OUT_L) << 0) | 
//           (LPSread(LPS22HB_TEMP_OUT_H) << 8)/100;

//   // to reduce loop time you don't have to request and read all the bytes in a loop
// }

// // void read_baro(){
// //     uint8_t buffer[5];
// //     LPSreadMultiple(LPS22HB_PRES_OUT_XL, buffer, 5);

// //     P = ((uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16)) / 40.96;
// //     T = ((uint16_t)buffer[3] | ((uint16_t)buffer[4] << 8)) / 100;
// // }