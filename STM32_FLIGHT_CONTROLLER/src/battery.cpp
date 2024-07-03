#include "battery.hpp"
#include "drone.hpp"

namespace battery
{
  int pin = PB1;
  float low = 11.1;
  float voltage;
  constexpr float referenceVoltage = 3.3;
  constexpr float adcMaxValue = 4095.0;
  constexpr float voltageDividerRatio = 11.0;
  constexpr float smoothingFactor = 0.08;

  void setup()
  {
    analogReadResolution(12);
    pinMode(pin, INPUT_ANALOG);
  }

  void read()
  {
    uint16_t adcValue = analogRead(pin);
    float temp_voltage = (referenceVoltage * adcValue / adcMaxValue) * voltageDividerRatio;
    voltage = voltage * (1 - smoothingFactor) + temp_voltage * smoothingFactor;
    
    if (voltage <= low)
    {
      drone::error = 7;
    }
  }
}
