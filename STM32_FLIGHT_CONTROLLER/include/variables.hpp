#include <Arduino.h>

#define RED_LED_PIN PB14
#define GREEN_LED_PIN PB12
#define BUZZER_PIN PB13

#define ADC_RESOLUTION 12 // 12-bit resolution for STM32F401CCU6

#define BAT_VOLTAGE_PIN PB1

#define ch1_out PB6
#define ch2_out PB7
#define ch3_out PB8
#define ch4_out PB9
#define inTimPin PB10

TIM_TypeDef *Instance_in = TIM2;
TIM_TypeDef *Instance_out = TIM4;

HardwareTimer *InTim = new HardwareTimer(Instance_in);
HardwareTimer *OutTim = new HardwareTimer(Instance_out);

uint32_t chx = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(inTimPin), PinMap_PWM));


TwoWire HWire (PB4, PA8);// might need to change to PB4 and PA8
HardwareSerial gpsSerial(PA10, PA9);


// SPIClass nrfSPI(PA7, PA6, PA5);
// const SPISettings nrfSPISettings(10000000, MSBFIRST, SPI_MODE0);
// SPI_HandleTypeDef hspi1;
// #define NRF24_SPI &hspi1


bool auto_level = true;                 //Auto level on (true) or off (false).

float pid_p_gain_roll = 1.3;               //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.04;              //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 18.0;              //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).

float pid_p_gain_altitude = 1.4;           //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude = 0.2;           //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude = 0.75;          //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-).

//During flight the battery voltage drops and the motors are spinning at a lower RPM. This has a negative effecct on the
//altitude hold function. With the battery_compensation variable it's possible to compensate for the battery voltage drop.
//Increase this value when the quadcopter drops due to a lower battery voltage during a non altitude hold flight.
float battery_compensation = 40.0;

float declination = -1.4;                   //Set the declination between the magnetic and geographic north.

int16_t manual_takeoff_throttle = 0;    //Enter the manual hover point when auto take-off detection is not desired (between 1400 and 1600).
int16_t motor_idle_speed = 1100;           //Enter the minimum throttle pulse of the motors when they idle (between 1000 and 1200). 1170 for DJI

uint8_t gyro_address = 0x68;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
uint8_t compass_address = 0x1E;            //The I2C address of the HMC5883L is 0x1E in hexadecimal form.
uint8_t eeprom_address = 0x50;            //The I2C address of the EEPROM is 0x50 in hexadecimal form.
uint8_t barometer_address = 0x5D;            //The I2C address of the LPS22HB is 0x5D in hexadecimal form.

float low_battery_warning = 10.5;          //Set the battery warning at 10.5V (default = 10.5V).

#define STM32_board_LED PC13               //Change PC13 if the LED on the STM32 is connected to another output.

//Tuning parameters/settings is explained in this video: https://youtu.be/ys-YpOaA2ME
#define variable_1_to_adjust dummy_float   //Change dummy_float to any setting that you want to tune.
#define variable_2_to_adjust dummy_float   //Change dummy_float to any setting that you want to tune.
#define variable_3_to_adjust dummy_float   //Change dummy_float to any setting that you want to tune.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t check_byte, flip32, start, armed, ready;
uint8_t error, error_counter, error_led;
uint8_t flight_mode, flight_mode_counter, flight_mode_led;
uint8_t takeoff_detected, manual_altitude_change;
uint8_t telemetry_send_byte, telemetry_bit_counter, telemetry_loop_counter;

uint32_t telemetry_send_payload1, telemetry_send_payload2;
int32_t telemetry_send_payload3, telemetry_send_payload4; 
float telemetry_send_payload5, telemetry_send_payload6;

uint8_t telemetry_send_signature;
uint8_t channel_select_counter;
uint8_t level_calibration_on;

uint32_t telemetry_buffer_byte;

int16_t esc_1 = 1000, esc_2 = 1000, esc_3 = 1000, esc_4 = 1000;
int16_t manual_throttle;
int16_t throttle, takeoff_throttle, cal_int;
int16_t temperature, count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;

int32_t channel_1_start, channel_1, channel_1_base, pid_roll_setpoint_base;
int32_t channel_2_start, channel_2, channel_2_base, pid_pitch_setpoint_base;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t channel_7_start, channel_7;
int32_t channel_8_start, channel_8;
int32_t channel_9_start, channel_9;
int32_t channel_10_start, channel_10;
int32_t measured_time, measured_time_start, receiver_watchdog;
int32_t acc_total_vector, acc_total_vector_at_start;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int16_t acc_pitch_cal_value, acc_y_cal;
int16_t acc_roll_cal_value, acc_x_cal;
int32_t max_angle;

int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total ;
int16_t acc_z_average_short[26], acc_z_average_long[51];

uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;

int32_t acc_alt_integrated;

uint32_t loop_timer, error_timer, flight_mode_timer, loop_time_prev, loop_time_actual;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error, p_term_roll, i_term_roll, d_term_roll;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error, p_term_pitch, i_term_pitch, d_term_pitch;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error, p_term_yaw, i_term_yaw, d_term_yaw;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
float battery_voltage, dummy_float;
float drone_roll_state, drone_pitch_state;



//Telemetry variables
#define CEPIN PB0 
#define CSNPIN PA4


RF24 radio(CEPIN, CSNPIN); // CE, CSN on Blue Pill 
const byte addresses[][6] = {"00001", "00002"};
// uint8_t TxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA};
// uint8_t TxData[32] = "Hello World\n";//{0xEE,0xDD,0xCC,0xBB,0xAA};

 
struct TXData 
{
  uint8_t signature;
  uint32_t payload1;
  uint32_t payload2;
  int32_t payload3;
  int32_t payload4;
  float payload5;
  float payload6;
};

struct RXData 
{
  uint8_t signature;
  uint32_t payload1;
  uint32_t payload2;
  int32_t payload3;
  int32_t payload4;
  float payload5;
  float payload6;
};

TXData data_tx;
RXData data_rx;
