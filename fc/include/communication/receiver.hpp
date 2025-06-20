#ifndef RECEIVER_HPP
#define RECEIVER_HPP
#include <Arduino.h>

namespace receiver
{
    extern uint16_t channels[10]; // Array to store channels
    extern uint8_t channel_select_counter;
    extern int32_t measured_time, measured_time_start;

    extern int pin;
    extern hw_timer_t *timer;
    extern portMUX_TYPE timerMux;

    extern uint16_t &pitch, &roll, &throttle, &yaw;
    extern uint16_t &vra, &vrb;
    extern uint16_t &switch_a, &switch_b, &switch_c, &switch_d;

    void setup();
    void IRAM_ATTR updateChannels();
    void wait();
}

#endif // !RECEIVER_HPP