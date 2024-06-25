#include "led.hpp"

namespace led
{
  int pin = PC13;

  void setup()
  {
    pinMode(pin, OUTPUT);
  }

  void blink()
  {
    uint8_t count = 0;
    for (count = 0; count < 1250; ++count)
    {
      if (count % 125 == 0)
      {
        digitalWrite(pin, !digitalRead(pin));
      }
      delay(4);
    }
    count = 0;
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