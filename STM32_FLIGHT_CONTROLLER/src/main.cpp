#include <Arduino.h>
#include "uav/i2c_utils.hpp"
#include "uav/drone.hpp"
#include "uav/flight.hpp"
#include "uav/eeprom.hpp"
#include "uav/led.hpp"
#include "uav/battery.hpp"

#include "attitude_estimation/attitude_estimation.hpp"

#include "communication/telemetry.hpp"
#include "communication/receiver.hpp"
#include "communication/raspberrypi.hpp"
,
#include "controllers/pid.hpp"

#include "actuators/motors.hpp"
#include "actuators/outputs.hpp"
#include "actuators/control.hpp"

AttitudeEstimator attitude_estimator;

void setup()
{
    // rpi::setup();
    led::setup();
    drone::setup();
    flight::setup();
    battery::setup();
    i2c::setup();
    receiver::setup();
    motors::setup();
    telemetry::setup();
    attitude_estimator.setup();

    led::off();
}

void loop()
{
    attitude_estimator.update();

    battery::read();

    flight::update();

    motors::set_speed();

    // telemetry::send();
    // rpi::send();

    // telemetry::loop();
    
    drone::wait();
}
