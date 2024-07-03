#include "flight.hpp"
#include "receiver.hpp"
#include "gyro.hpp"
#include "pid_controller.hpp"
#include "motors.hpp"

namespace flight
{
    // FLIGH_MODES = ["STANDBY", "READY", "ACRO", "LEVEL", "ALT_HOLD"];

    uint8_t start, armed, mode;

    void setup()
    {
        mode = 0;
    }

    void update()
    {
        if (receiver::switch_a < 1500 && receiver::throttle < 1050)
        {
            mode = 0;
            start = 0;
        }

        if (receiver::switch_a > 1500 && receiver::throttle < 1050 && mode == 0)
        {
            mode = 1;
            start = 1;
        }

        if (mode == 1 && receiver::throttle < 1050 && receiver::roll > 1400 && receiver::roll < 1600 && receiver::pitch > 1400 && receiver::pitch < 1600 && receiver::yaw > 1400 && receiver::yaw < 1600)
        {
            mode = 3;
            start = 2;
            gyro::pitch = gyro::pitch_accelerometer;
            gyro::roll = gyro::roll_accelerometer;

            pid::reset();
        }

        if (mode != 0 && mode != 2)
        {
            pid::calculate();
            motors::running();
        }

        else
        {
            motors::off();
        }
    }
}
