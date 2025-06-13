#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <Arduino.h>

namespace motors
{
    // Constants for LEDC PWM setup
    extern const int freq;
    extern const int resolution;
    
    // Output pins for each motor
    extern const int ch1_pin; // FR
    extern const int ch2_pin; // FL
    extern const int ch3_pin; // BR
    extern const int ch4_pin; // BL
    
    // LEDC channels for each motor
    extern const int ch1_channel;
    extern const int ch2_channel;
    extern const int ch3_channel;
    extern const int ch4_channel;
    
    // Motor speed variables
    extern int16_t fr, fl, br, bl;

    void setup();
    void set_speed();
    void running();
    void off();
    void idle();
}

#endif // MOTORS_HPP