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

#include "controllers/pid.hpp"

#include "actuators/motors.hpp"
#include "actuators/outputs.hpp"

AttitudeEstimator attitude_estimator;

void setup()
{
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
    drone::loop();

    attitude_estimator.update();

    battery::read();

    flight::update();

    telemetry::send();
    telemetry::loop();
    motors::set_speed();
    drone::wait();
}
