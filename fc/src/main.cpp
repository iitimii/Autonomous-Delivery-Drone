#include <Arduino.h>
#include "uav/i2c_utils.hpp"
#include "uav/drone.hpp"
#include "uav/flight.hpp"
#include "uav/eeprom.hpp"
#include "uav/led.hpp"
#include "uav/battery.hpp"
#include "uav/eeprom.hpp"

#include "state_estimation/state_estimation.hpp"

#include "communication/telemetry.hpp"
#include "communication/receiver.hpp"
#include "communication/debugging.hpp"

#include "controllers/pid.hpp"

#include "actuators/motors.hpp"
#include "actuators/outputs.hpp"
#include "actuators/control.hpp"




StateEstimator state_estimator;
bool debug = true;
// bool debug = false;
uint32_t counter = 0;

void setup()
{
    debugging::setup(debug);
    eeprom::setup();
    led::setup();
    drone::setup();
    flight::setup();
    battery::setup();
    i2c::setup();
    receiver::setup();
    motors::setup();
    state_estimator.setup();
    led::off();
}

void loop()
{
    state_estimator.update();

    battery::read();

    flight::update();

    motors::set_speed();

    String message = "";
    // message = "Battery: " + String(battery::voltage) + "\n";
    // message += "Flight Mode: " + String(flight::mode) + "\n";
    // message += "Roll: " + String(state_estimator.roll) + " Pitch: " + String(state_estimator.pitch) + " Yaw: " + String(state_estimator.yaw) + "\n";
    // message += "gx: " + String(state_estimator.gx) + " gy: " + String(state_estimator.gy) + " gz: " + String(state_estimator.gz) + "\n";
    // message += " ax: " + String(state_estimator.ax) + " ay: " + String(state_estimator.ay) + " az: " + String(state_estimator.az) + "\n";
    // message += "Heading: " + String(state_estimator.heading) + "\n";
    // message += "Latitude: " + String(state_estimator.latitude, 6U) + " Longitude: " + String(state_estimator.longitude, 6U) + " Num Sats: " + String(state_estimator.num_satelites) + "\n";
    // message += "Altitude: " + String(state_estimator.altitude) + "\n";
    // message += "x: " + String(state_estimator.x) + " x_dot: " + String(state_estimator.x_dot) + " x_d_dot: " + String(state_estimator.x_d_dot) + "\n";
    // message += "y: " + String(state_estimator.y) + " y_dot: " + String(state_estimator.y_dot) + " y_d_dot: " + String(state_estimator.y_d_dot) + "\n";
    // message += "z: " + String(state_estimator.z) + " z_dot: " + String(state_estimator.z_dot) + " z_d_dot: " + String(state_estimator.z_d_dot) + "\n";
    // message += "Roll Output: " + String(outputs::roll) + " Pitch Output: " + String(outputs::pitch) + " Yaw Output: " + String(outputs::yaw) + " Throttle Output: " + String(outputs::throttle) + "\n";
    // message += "Channel 1: " + String(receiver::roll) + " Channel 2: " + String(receiver::pitch) + " Channel 3: " + String(receiver::throttle) + " Channel 4: " + String(receiver::yaw) + " Channel 5: " + String(receiver::switch_a) + " Channel 6: " + String(receiver::switch_b) + " Channel 7: " + String(receiver::switch_c) + " Channel 8: " + String(receiver::switch_d) + " Channel 9: " + String(receiver::vra) + " Channel 10: " + String(receiver::vrb);
    // message += "Position (x, y, z): (" + String(state_estimator.x) + ", " + String(state_estimator.y) + ", " + String(state_estimator.z) + "\n";
    // message += "\nVelocity (vx, vy, vz): (" + String(state_estimator.x_dot) + ", " + String(state_estimator.y_dot) + ", " + String(state_estimator.z_dot) + "\n";
    // message += "\nOrientation (roll, pitch, yaw): (" + String(state_estimator.phi) + ", " + String(state_estimator.theta) + ", " + String(state_estimator.psi) + "\n";
    // message += "\nAngular velocity (p, q, r): (" + String(state_estimator.p) + ", " + String(state_estimator.q) + ", " + String(state_estimator.r) + "\n";
    message += "Position x: " + String(state_estimator.x, 2U) + " y: " + String(state_estimator.y, 2U) + " z: " + String(state_estimator.z, 2U) + "\n";
    message += "Velocity x_dot: " + String(state_estimator.x_dot, 2U) + " y_dot: " + String(state_estimator.y_dot, 2U) + " z_dot: " + String(state_estimator.z_dot, 2U) + "\n";
    message += "Orientation roll: " + String(state_estimator.phi, 2U) + " pitch: " + String(state_estimator.theta, 2U) + " yaw: " + String(state_estimator.psi, 2U) + "\n";
    message += "Angular velocity p: " + String(state_estimator.p, 2U) + " q: " + String(state_estimator.q, 2U) + " r: " + String(state_estimator.r, 2U) + "\n";
    

    if ((counter % 30)==0)
        debugging::log(message);
    ++counter;
    
    drone::wait();
}
