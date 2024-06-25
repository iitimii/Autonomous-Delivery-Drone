#include "battery.hpp"

namespace battery
{

  int pin = PB1;
  int adc_res = 12;
  float low_warning = 11.1;
  float voltage;

  void setup()
  {
    analogReadResolution(adc_res);    
    pinMode(pin, INPUT_ANALOG);
  }

  float read()
  {
    uint16_t adcValue = analogRead(pin);
    float voltage = 3.3 * adcValue / 4095; 
    return voltage * 11.0;                 
  }

}