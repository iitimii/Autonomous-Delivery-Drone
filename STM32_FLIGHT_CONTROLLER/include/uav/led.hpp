#ifndef LED_CONTROL_HPP
#define LED_CONTROL_HPP
#include <Arduino.h>
namespace led
{
    extern int pin;

    void setup();
    void blink_once();
    void blink();
    void on();
    void off();
}

#endif // !LED_CONTROL_HPP