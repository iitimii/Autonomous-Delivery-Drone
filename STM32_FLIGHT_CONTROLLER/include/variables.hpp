#include <Arduino.h>

#define BAT_VOLTAGE_PIN PB1
#define ch1_out PB6
#define ch2_out PB7
#define ch3_out PB8
#define ch4_out PB9
#define inTimPin PB10

// PID Variables
float dt = 0.004;
int roll_vel_setpoint, pitch_vel_setpoint, yaw_vel_setpoint;
int pid_roll_vel_output, pid_pitch_vel_output, pid_yaw_vel_output;
float kp_roll_vel = 1, ki_roll_vel = 1, kd_roll_vel = 1;
float kp_pitch_vel = 1, ki_pitch_vel = 1, kd_pitch_vel = 1;
float kp_yaw_vel = 1, ki_yaw_vel = 1, kd_yaw_vel = 1;




//I2C Variables
TwoWire HWire(PB4, PA8); // might need to change to PB4 and PA8
uint8_t gyro_address = 0x68;    // The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
uint8_t compass_address = 0x1E; // The I2C address of the HMC5883L is 0x1E in hexadecimal form.
uint8_t eeprom_address = 0x50;  // The I2C address of the EEPROM is 0x50 in hexadecimal form.
uint8_t baro_address = 0x76;    // The I2C address of the LPS22HB is 0x5D in hexadecimal form.


//System Variables
int adc_res{12}; // 12-bit resolution for STM32F401CCU6
#define STM32_board_LED PC13 // Change PC13 if the LED on the STM32 is connected to another output.


// Battery variables
float low_battery_warning = 10.5; // Set the battery warning at 10.5V (default = 10.5V).
float battery_voltage;


// Flight variables
uint8_t start, armed, ready;
uint8_t error;
uint8_t flight_mode;
uint8_t takeoff_detected, manual_altitude_change;
uint16_t count_var;
uint32_t loop_timer, loop_time_prev, loop_time_actual;
bool auto_level = true; // Auto level on (true) or off (false).
float declination = -1.4; // Set the declination between the magnetic and geographic north.

// Gyro variables
int16_t acc_x, acc_y, acc_z;
int32_t acc_resultant;
int16_t roll_velocity, pitch_velocity, yaw_velocity;
int32_t roll_vel_cal, pitch_vel_cal, yaw_vel_cal;
float roll_angle_acc, pitch_angle_acc, pitch_angle, roll_angle;
float roll_velocity_lpf, pitch_velocity_lpf, yaw_velocity_lpf;
int16_t temperature;



// Motor variables
int16_t motor_fr=1000, motor_fl=1000, motor_br=1000, motor_bl=1000;
int16_t manual_throttle;
int16_t throttle, takeoff_throttle, cal_int;

// Receiver variables
TIM_TypeDef *Instance_in = TIM2;
TIM_TypeDef *Instance_out = TIM4;
HardwareTimer *InTim = new HardwareTimer(Instance_in);
HardwareTimer *OutTim = new HardwareTimer(Instance_out);
uint32_t chx = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(inTimPin), PinMap_PWM));
uint8_t channel_select_counter;
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


// Telemetry variables
#define CEPIN PB0
#define CSNPIN PA4
RF24 radio(CEPIN, CSNPIN); // CE, CSN on Blue Pill
const byte addresses[][6] = {"00001", "00002"};
uint8_t telemetry_send_byte, telemetry_loop_counter;
uint32_t telemetry_send_payload1, telemetry_send_payload2;
int32_t telemetry_send_payload3, telemetry_send_payload4;
float telemetry_send_payload5, telemetry_send_payload6;
uint8_t telemetry_send_signature;
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