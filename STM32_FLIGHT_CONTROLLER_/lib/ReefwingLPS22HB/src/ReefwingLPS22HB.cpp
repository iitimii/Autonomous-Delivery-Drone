/******************************************************************
  @file       ReefwingLPS22HB.cpp
  @brief      Arduino Library for the LPS22HB Pressure Sensor
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.3
  Date:        21/02/23

  1.0.1 Original Release.                         11/02/23
  1.0.2 Added 2's comp for temperature            20/02/23
  1.0.3 Fixed setODR() bug                        21/02/23

  Credit - Some code used from the LPS22HB Absolute Digital Barometer 
           class  by Adrien Chapelet for IoThings.
           ref: https://github.com/adrien3d/IO_LPS22HB

******************************************************************/

#include <Arduino.h>
#include <Wire.h>

#include "ReefwingLPS22HB.h"

/******************************************************************
 *
 * Barometer Registers -
 * 
 ******************************************************************/

#define LPS22HB_WHO_AM_I        0X0F // Who am I
#define LPS22HB_RES_CONF        0X1A // Normal (0) or Low current mode (1)
#define LPS22HB_CTRL_REG1       0X10 // Output rate and filter settings
#define LPS22HB_CTRL_REG2       0X11 // BOOT FIFO_EN STOP_ON_FTH IF_ADD_INC I2C_DIS SWRESET One_Shot
#define LPS22HB_STATUS_REG      0X27 // Temp or Press data available bits
#define LPS22HB_PRES_OUT_XL     0X28 // XLSB
#define LPS22HB_PRES_OUT_L      0X29 // LSB
#define LPS22HB_PRES_OUT_H      0X2A // MSB
#define LPS22HB_TEMP_OUT_L      0X2B // LSB
#define LPS22HB_TEMP_OUT_H      0X2C // MSB
#define LPS22HB_WHO_AM_I_VALUE  0xB1 // Expected return value of WHO_AM_I register

/******************************************************************
 *
 * I2C Device Addresses - 
 * 
 ******************************************************************/

#define LPS22HB_ADDRESS   0x5C  //  Address of barometer (Nano 33 BLE SENSE) 
#define DEFAULT_ADDRESS   0x5D  //  Sensor I2C Default Address

/******************************************************************
 *
 * Pressure Reference - See README
 * 
 ******************************************************************/

#define STANDARD_ATMOSPHERE 1013.25   //  Average Sea-Level Pressure
#define ALTITUDE_EXPONENT   0.190266  //  exp = -(R * Lr) / gM
#define TEMP_LAPSE_RATE     0.0065    //  Lr = -0.0065 [K/m]
#define HPA_TO_MMHG         0.750062  //  Convert hPa to mmHg
#define HPA_TO_INHG         0.029530  //  Convert hPa to inHg

/******************************************************************
 *
 * LPS22HB Implementation - 
 * 
 ******************************************************************/

ReefwingLPS22HB::ReefwingLPS22HB() : 
  _qnhIsSet(false),
  _address(LPS22HB_ADDRESS),
  _rate((int)Rate::RATE_ONE_SHOT) { }

void ReefwingLPS22HB::begin() {
  Wire1.begin();

  //  Check that chip boot up is complete
  while ((read(LPS22HB_CTRL_REG2) & 0x07) != 0) {
    yield();
  }

  //  power-down mode, LPF off, and BDU on
  write(LPS22HB_CTRL_REG1, 0x02);   

  //  Save First Reading for QFE References
  firstReading.pressure = readPressure();
  firstReading.temperature = readTemperature(Scales::KELVIN);
}

void ReefwingLPS22HB::reset() {
  //  0x04 = 0100b, bit 2 = SWRESET, set to reset chip
  write(LPS22HB_CTRL_REG2, 0x04);   
  
  // software reset. Bit self clears when reset complete.
  while ((read(LPS22HB_CTRL_REG2) & 0x04) != 0) {
    yield();
  }
}

byte ReefwingLPS22HB::whoAmI() {
  Wire1.beginTransmission(_address);
  Wire1.write(LPS22HB_WHO_AM_I);
  Wire1.endTransmission();
  Wire1.requestFrom(_address, 1);
  return Wire1.read();
}

bool ReefwingLPS22HB::connected() {
  return (whoAmI() == LPS22HB_WHO_AM_I_VALUE);
}

void ReefwingLPS22HB::setODR(Rate rate) {
  _rate = (uint8_t)rate;

  //  Set ODR bits 4, 5 & 6 (_rate & 0x07) << 4 and BDU 0x02
  write(LPS22HB_CTRL_REG1, ((_rate & 0x07) << 4) | 0x02);
}

void ReefwingLPS22HB::setQNH(float q) {
  _qnh = q;
  _qnhIsSet = true;
}

float ReefwingLPS22HB::getQNH() {
  return _qnh;
}

void ReefwingLPS22HB::clearQNH() {
  _qnh = 0.0f;
  _qnhIsSet = false;
}

void ReefwingLPS22HB::triggerOneShot() {
  write(LPS22HB_CTRL_REG2, 0x01);

  // wait for ONE_SHOT bit to clear
  while ((read(LPS22HB_CTRL_REG2) & 0x01) != 0) {
    yield();
  }
}

float ReefwingLPS22HB::readPressure(Units units) {
  if (_rate == (uint8_t)Rate::RATE_ONE_SHOT) { triggerOneShot(); }

  //  To guarantee the correct behavior of the BDU feature, 
  //  PRESS_OUT_H (0x2A) must be the last address read.
  uint8_t pressOutXL = read(LPS22HB_PRES_OUT_XL);
  uint8_t pressOutL = read(LPS22HB_PRES_OUT_L);
  uint8_t pressOutH = read(LPS22HB_PRES_OUT_H);
  
  long val = ( ((long)pressOutH << 16) | ((long)pressOutL << 8) | (long)pressOutXL );
  float result = (double)val / 4096.0f;
  
  switch (units) {
    case Units::HECTOPASCAL:
    case Units::MILLIBAR:
      //  Sensor returns value in hPa = mbar = 0.1 kPa = 0.0145 PSI
      break;
    case Units::KILOPASCAL:
      result = result * 0.1f;
      break;
    case Units::PSI:
      result = result * 0.0145037738;
      break;
    case Units::ATMOSPHERES:
      result = result / STANDARD_ATMOSPHERE;
      break;
    case Units::MM_HG:
      result = result * HPA_TO_MMHG;
      break;
    case Units::IN_HG:
      result = result * HPA_TO_INHG;
  }

  return result;
}

int16_t ReefwingLPS22HB::twosCompToInteger(uint16_t two_compliment_val) {
    // [0x0000; 0x7FFF] corresponds to [0; 32,767]
    // [0x8000; 0xFFFF] corresponds to [-32,768; -1]
    // int16_t has the range [-32,768; 32,767]

    uint16_t sign_mask = 0x8000;

    if ( (two_compliment_val & sign_mask) == 0 ) {
      // positive number - do nothing
      return two_compliment_val;
    } 
    else {
      //  if negative invert all bits, add one, and add sign
      return -(~two_compliment_val + 1);
    }
}

uint32_t ReefwingLPS22HB::readPressureCount() {
  if (_rate == (uint8_t)Rate::RATE_ONE_SHOT) { triggerOneShot(); }
  
  uint8_t pressOutXL = read(LPS22HB_PRES_OUT_XL);
  uint8_t pressOutL = read(LPS22HB_PRES_OUT_L);
  uint8_t pressOutH = read(LPS22HB_PRES_OUT_H);

  long val = ( ((long)pressOutH << 16) | ((long)pressOutL << 8) | (long)pressOutXL );
  
  return (uint32_t)val;
}

float ReefwingLPS22HB::readTemperature(Scales scales) {
  if (_rate == (uint8_t)Rate::RATE_ONE_SHOT) { triggerOneShot(); }

  uint8_t tempOutL = read(LPS22HB_TEMP_OUT_L);
  uint8_t tempOutH = read(LPS22HB_TEMP_OUT_H);

  uint16_t count = (tempOutH << 8) | (tempOutL & 0xff); 
  int16_t val = twosCompToInteger(count);

  float result = ((float)val)/100.0f;   // In Celsius

  switch (scales){
    case Scales::CELSIUS:
      break;
    case Scales::KELVIN:
      result += 273.15;
      break;
    case Scales::FAHRENHEIT:
      result = result * 1.8 + 32;
      break;
  }

  return result;
}

float ReefwingLPS22HB::readAltitude(PressureReference Pr) {
  float result = 0.0f;

  switch(Pr) {
    case PressureReference::QNE: {
      //  Reference Temp = 288.15K = 15C
      //  Tr/Lr = 44330.8
      float P = readPressure();   //  Pressure in hPa

      result = 44330.8f * (pow(P/STANDARD_ATMOSPHERE, ALTITUDE_EXPONENT) - 1);
      }
      break;
    case PressureReference::QFE:
      if (firstReading.pressure > 0.0f) {
        float P = readPressure();

        result = (firstReading.temperature/TEMP_LAPSE_RATE) * (pow(P/firstReading.pressure, ALTITUDE_EXPONENT) - 1);
      }
      break;
    case PressureReference::QNH:
      if (_qnhIsSet) {
        float P = readPressure();
        float Tr = readTemperature(Scales::KELVIN);

        result = (Tr/TEMP_LAPSE_RATE) * (pow(P/_qnh, ALTITUDE_EXPONENT) - 1);
      }
      break;
  }

  return result;
}

//  I2C read and write byte

uint8_t ReefwingLPS22HB::read(uint8_t reg) {
  Wire1.beginTransmission(_address);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom(_address, 1);
  return Wire1.read();
}

void ReefwingLPS22HB::write(uint8_t reg, uint8_t data) {
  Wire1.beginTransmission(_address);
  Wire1.write(reg);
  Wire1.write(data);
  Wire1.endTransmission();
}


