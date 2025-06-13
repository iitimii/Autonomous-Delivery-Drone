#include "uav/led.hpp"

namespace led
{
  void setup()
  {
    pinMode(LED_BUILTIN, OUTPUT);
    off();
  }

  void blink_once()
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  void blink()
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  void off()
  {
    digitalWrite(LED_BUILTIN, LOW);
  }

  void on()
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}