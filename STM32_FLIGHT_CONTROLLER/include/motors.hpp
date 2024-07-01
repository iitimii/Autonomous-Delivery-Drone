#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <Arduino.h>

namespace motors
{
    extern TIM_TypeDef *Instance_out;
    extern HardwareTimer *OutTim;

    extern int16_t fr, fl, br, bl;

    void setup();
    void set_speed();
    void running();
    void off();
    void idle();
}

#endif // MOTORS_HPP
