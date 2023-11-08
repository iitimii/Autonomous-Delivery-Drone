#include <Arduino.h>
#include <Wire.h>

#define PSCL PB10
#define PSDA PB3
TwoWire HWire (PSDA, PSCL); 

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
#define FIFO_CTRL 0x14 // FIFO


#define LPS22HB_ADDRESS   0x5D  //  Address of barometer 


#define STANDARD_ATMOSPHERE 1013.25   //  Average Sea-Level Pressure
#define ALTITUDE_EXPONENT   0.190266  //  exp = -(R * Lr) / gM
#define TEMP_LAPSE_RATE     0.0065    //  Lr = -0.0065 [K/m]
#define HPA_TO_MMHG         0.750062  //  Convert hPa to mmHg
#define HPA_TO_INHG         0.029530  //  Convert hPa to inHg


uint8_t LPSread(uint8_t reg) {
  HWire.beginTransmission(LPS22HB_ADDRESS);
  HWire.write(reg);
  HWire.endTransmission();
  HWire.requestFrom(LPS22HB_ADDRESS, 1);
  return HWire.read();
}

void LPSwrite(uint8_t reg, uint8_t data) {
  HWire.beginTransmission(LPS22HB_ADDRESS);
  HWire.write(reg);
  HWire.write(data);
  HWire.endTransmission();
}


float readPressure() {
  float reading = (LPSread(LPS22HB_PRES_OUT_XL) |
            (LPSread(LPS22HB_PRES_OUT_L) << 8) |
            (LPSread(LPS22HB_PRES_OUT_H) << 16)) / 40.96;
  
  return reading;
}


float readTemperature() {
float reading = (LPSread(LPS22HB_TEMP_OUT_L) << 0) | 
          (LPSread(LPS22HB_TEMP_OUT_H) << 8);

  return reading/100;
}




void baro_setup(){
    Serial.begin(57600);
    HWire.begin();
    delayMicroseconds(10);
while((LPSread(LPS22HB_CTRL_REG2) & 0x07) != 0) {
    yield();
  }
	LPSwrite(LPS22HB_CTRL_REG1, 0b01001110); // 50Hz
}

uint8_t count;
float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1;
float P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure, return_to_home_decrease;
int32_t dT, dT_C5;
//Altitude PID variables
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;

void baro_loop(){
  P = readPressure();
  pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                                //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;

    //To get better results we will use a complementary fillter that can be adjusted by the fast average.
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
    //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;     

  Serial.println(actual_pressure);
  delay(30);
}