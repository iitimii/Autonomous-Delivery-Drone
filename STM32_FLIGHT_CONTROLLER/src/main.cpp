#include <Arduino.h>
#include "i2c_utils.hpp"
#include "drone.hpp"
#include "flight.hpp"
#include "gyro.hpp"
#include "telemetry.hpp"
#include "receiver.hpp"
#include "led.hpp"
#include "pid_controller.hpp"
#include "battery.hpp"
#include "motors.hpp"
#include "outputs.hpp"
#include "eeprom.hpp"
// TODO if !read or send like 10 times, restart the device

void setup()
{
    drone::setup();
    flight::setup();
    battery::setup();
    led::setup();
    i2c::setup();
    telemetry::setup();
    receiver::setup();
    motors::setup();
    gyro::setup();
    pid::setup();

    motors::off();
    gyro::calibrate();
    battery::read();
}

void loop()
{
    drone::loop();

    gyro::read();
    gyro::calculate_attitude();

    battery::read();

    flight::update();

    telemetry::send();

    drone::wait();
}
