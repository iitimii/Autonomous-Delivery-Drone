#include "battery.hpp"

namespace battery
{

  int pin = PB1;
  float low = 11.1;
  float voltage;

  void setup()
  {
    analogReadResolution(12);    
    pinMode(pin, INPUT_ANALOG);
  }

  void read()
  {
    uint16_t adcValue = analogRead(pin);
    voltage = 3.3 * adcValue / 4095; 
    voltage *= 11.0;                 
  }

}