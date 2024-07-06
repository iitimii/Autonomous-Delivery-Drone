#include "uav/drone.hpp"
#include <Arduino.h>


namespace drone
{
        uint8_t error;
        uint16_t count_var;
        uint32_t loop_time, loop_time_prev, loop_time_actual;

        void setup()
        {
                error = 0;
        }

        void loop()
        {
                loop_time = micros();
                loop_time_prev = micros();
        }

        void wait()
        {
                loop_time_actual = micros() - loop_time_prev;
                while (micros() - loop_time < 4000)
                        ;
        }
}