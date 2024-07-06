#include "uav/led.hpp"

namespace led
{
  int pin = PC13;

  void setup()
  {
    pinMode(pin, OUTPUT);
    off();
  }

  void blink_once()
  {
    digitalWrite(pin, !digitalRead(pin));
  }

  void blink()
  {
    digitalWrite(pin, !digitalRead(pin));
  }

  void on()
  {
    digitalWrite(pin, LOW);
  }

  void off()
  {
    digitalWrite(pin, HIGH);
  }
}