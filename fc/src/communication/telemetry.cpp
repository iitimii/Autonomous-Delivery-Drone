// #include "communication/telemetry.hpp"
// #include "uav/led.hpp"
// #include "sensors/gps.hpp"
// #include "uav/drone.hpp"
// #include "uav/flight.hpp"
// #include "sensors/compass.hpp"
// #include "uav/battery.hpp"
// #include "sensors/barometer.hpp"
// #include "communication/receiver.hpp"
// #include "actuators/motors.hpp"
// #include "actuators/control.hpp"
// #include "actuators/outputs.hpp"
// #include "attitude_estimation/attitude_estimation.hpp"

// namespace telemetry
// {
//   int irq_pin = PA1;
//   RF24 radio(PB0, PA4);
//   const byte addresses[][6] = {"00001", "00002"};

//   RadioData data_tx;
//   RadioData data_rx;

//   volatile bool dataReceived = false;

//   void setup()
//   {
//     radio.begin();
//     radio.setChannel(120);
//     radio.setDataRate(RF24_2MBPS);
//     radio.setPALevel(RF24_PA_MAX);
//     radio.setAutoAck(false);
//     radio.disableDynamicPayloads();
//     radio.maskIRQ(1, 1, 0);
//     attachInterrupt(digitalPinToInterrupt(irq_pin), interrupt, FALLING);
//     radio.openWritingPipe(addresses[0]);
//     radio.openReadingPipe(0, addresses[1]);
//     radio.startListening();
//   }

//   void select_data()
//   {
//     static uint8_t telemetry_loop_counter = 0;

//     switch (telemetry_loop_counter)
//     {
//     case 0:
//       data_tx.signature = 'T';                    // uint8_t
//       data_tx.payload1 = drone::loop_time_actual; // int16_t
//       data_tx.payload2 = drone::error;            // int16_t
//       data_tx.payload3 = flight::start;           // int16_t
//       data_tx.payload4 = battery::voltage;
//       data_tx.payload5 = attitude_estimator.pitch;
//       data_tx.payload6 = attitude_estimator.roll;
//       data_tx.payload7 = attitude_estimator.yaw;
//       data_tx.payload8 = attitude_estimator.ay;
//       data_tx.payload9 = attitude_estimator.ax;
//       break;

//     case 1:
//       data_tx.signature = 'I';
//       data_tx.payload1 = flight::armed;
//       data_tx.payload2 = flight::mode;
//       data_tx.payload3 = compass::heading_lock;
//       data_tx.payload4 = attitude_estimator.gy;
//       data_tx.payload5 = attitude_estimator.gx;
//       data_tx.payload6 = attitude_estimator.gz;
//       data_tx.payload7 = gps::latitude;
//       data_tx.payload8 = gps::longitude;
//       data_tx.payload9 = compass::compass_heading;
//       break;

//     case 2:
//       data_tx.signature = 'M';
//       data_tx.payload1 = gps::num_satelites;
//       data_tx.payload2 = gps::fix_type;
//       data_tx.payload3 = attitude_estimator.acc_resultant;
//       data_tx.payload4 = attitude_estimator.pressure;
//       // data_tx.payload5 = baro::altitude;
//       data_tx.payload6 = attitude_estimator.temperature;
//       data_tx.payload7 = receiver::roll;
//       data_tx.payload8 = receiver::pitch;
//       data_tx.payload9 = receiver::throttle;
//       break;

//     case 3:
//       data_tx.signature = 'L';
//       data_tx.payload1 = receiver::yaw;
//       data_tx.payload2 = receiver::switch_a;
//       data_tx.payload3 = receiver::switch_b;
//       data_tx.payload4 = receiver::vra;
//       data_tx.payload5 = receiver::switch_c;
//       data_tx.payload6 = receiver::vrb;
//       data_tx.payload7 = receiver::switch_d;
//       data_tx.payload8 = outputs::pitch;
//       data_tx.payload9 = outputs::roll;
//       break;

//     case 4:
//       data_tx.signature = 'H';
//       data_tx.payload1 = motors::br;
//       data_tx.payload2 = motors::bl;
//       data_tx.payload3 = outputs::throttle;
//       data_tx.payload4 = outputs::yaw;
//       data_tx.payload5 = outputs::angle.pitch;
//       data_tx.payload6 = outputs::angle.roll;
//       data_tx.payload7 = control::pitch_rate_gain.kp;
//       data_tx.payload8 = control::pitch_rate_gain.ki;
//       data_tx.payload9 = control::pitch_rate_gain.kd;
//       break;

//     case 5:
//       data_tx.signature = 'N';
//       data_tx.payload1 = motors::fl;
//       data_tx.payload2 = motors::fr;
//       data_tx.payload3 = battery::percentage;
//       data_tx.payload4 = control::roll_rate_gain.kp;
//       data_tx.payload5 = control::roll_rate_gain.ki;
//       data_tx.payload6 = control::roll_rate_gain.kd;
//       data_tx.payload7 = control::yaw_rate_gain.kp;
//       data_tx.payload8 = control::yaw_rate_gain.ki;
//       data_tx.payload9 = control::yaw_rate_gain.kd;
//       break;

//     case 6:
//       data_tx.signature = 'O';
//       data_tx.payload1 = 0;
//       data_tx.payload2 = 0;
//       data_tx.payload3 = 0;
//       data_tx.payload4 = control::pitch_angle_gain.kp;
//       data_tx.payload5 = control::pitch_angle_gain.ki;
//       data_tx.payload6 = control::pitch_angle_gain.kd;
//       data_tx.payload7 = control::roll_angle_gain.kp;
//       data_tx.payload8 = control::roll_angle_gain.ki;
//       data_tx.payload9 = control::roll_angle_gain.kd;
//       break;

//     case 7:
//       data_tx.signature = 'W'; // uint8_t
//       data_tx.payload1 = 0;    // int16_t
//       data_tx.payload2 = 0;    // int16_t
//       data_tx.payload3 = 0;    // int16_t
//       data_tx.payload4 = attitude_estimator.az;
//       data_tx.payload5 = 0.0f;
//       data_tx.payload6 = 0.0f;
//       data_tx.payload7 = 0.0f;
//       data_tx.payload8 = 0.0f;
//       data_tx.payload9 = 0.0f;
//       break;

//     default:
//       data_tx.signature = 'E'; // uint8_t
//       data_tx.payload1 = 0;    // int16_t
//       data_tx.payload2 = 0;    // int16_t
//       data_tx.payload3 = 0;    // int16_t
//       data_tx.payload4 = 0.0f;
//       data_tx.payload5 = 0.0f;
//       data_tx.payload6 = 0.0f;
//       data_tx.payload7 = 0.0f;
//       data_tx.payload8 = 0.0f;
//       data_tx.payload9 = 0.0f;
//       break;
//     }

//     telemetry_loop_counter++;
//     if (telemetry_loop_counter > 7)
//       telemetry_loop_counter = 0;
//   }

//   void send()
//   {
//     static uint8_t failureCount = 0;
//     radio.stopListening();
//     select_data();
//     bool success = radio.write(&data_tx, sizeof(data_tx));

//     if (!success)
//     {
//       ++failureCount;
//       if (failureCount >= 1500)
//       {
//         setup();
//         failureCount = 0;
//       }
//     }
//     else
//     {
//       failureCount = 0;
//     }

//     radio.startListening();
//   }

//   void interrupt()
//   {
//     dataReceived = true;
//   }

//   void loop()
//   {
//     if (dataReceived)
//     {
//       dataReceived = false;
//       while (radio.available())
//       {
//         radio.read(&data_rx, sizeof(RadioData));
//       }
//     }
//   }

// }