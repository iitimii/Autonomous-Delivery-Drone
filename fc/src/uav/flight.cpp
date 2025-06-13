#include "uav/flight.hpp"
#include "communication/receiver.hpp"
#include "state_estimation/state_estimation.hpp"
#include "actuators/control.hpp"
#include "actuators/motors.hpp"
#include "uav/battery.hpp"

namespace flight
{
    // FLIGH_MODES = ["STANDBY", "READY", "ACRO", "LEVEL", "VEL_CONTROL"];

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

            if (battery::voltage < 11.1)
            {
                mode = 0;
                start = 0;
                armed = 0;
                state_estimator.reset();
                control::reset_controllers();
                motors::off();
            }
        }

        if (mode == 1 && receiver::throttle < 1050 && receiver::roll > 1400 && receiver::roll < 1600 && receiver::pitch > 1400 && receiver::pitch < 1600 && receiver::yaw > 1400 && receiver::yaw < 1600)
        {
            mode = 3;
            start = 2;

            state_estimator.reset();
            control::reset_controllers();
        }

        if (mode >= 3)
        {
            control::attitude_control();
            motors::running();
        }

        // if (mode == 4)
        // {
        //     // control::velocity_control();
        //     // motors::running();
        // }

        else
        {
            motors::off();
        }
    }
}
