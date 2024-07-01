#ifndef DRONE_HPP
#define DRONE_HPP
#include <Arduino.h>

namespace drone
{

    extern uint8_t error;
    extern uint16_t count_var;
    extern uint32_t loop_time, loop_time_prev, loop_time_actual;

}


#endif // !DRONE_HPP