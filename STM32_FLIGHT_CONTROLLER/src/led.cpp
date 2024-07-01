#include "led.hpp"

namespace led
{
  int pin = PC13;

  void setup()
  {
    pinMode(pin, OUTPUT);
  }

  void blink_once()
  {
    digitalWrite(pin, !digitalRead(pin));
  }

  void blink()
  {
    uint8_t count = 0;
    for (count = 0; count < 1250; ++count)
    {
      if (count % 125 == 0)
      {
        blink_once();
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