#include "uav/flight.hpp"
#include "communication/receiver.hpp"
#include "attitude_estimation/attitude_estimation.hpp"
#include "actuators/control.hpp"
#include "actuators/motors.hpp"

namespace flight
{
    // FLIGH_MODES = ["STANDBY", "READY", "ACRO", "LEVEL", "ALT_HOLD"];

    uint8_t start, mode;
    uint16_t armed;

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

            attitude_estimator.reset();
            control::reset_controllers();
        }

        if (mode != 0 && mode != 2)
        {
            control::calculate();
            motors::running();
        }

        else
        {
            motors::off();
        }
    }
}
