void battery_setup() {
  pinMode(BAT_VOLTAGE_PIN, INPUT_ANALOG); // Configure PB1 as analog input
  analogReadResolution(adc_res); // Set ADC resolution to 12-bit
}


float get_voltage() 
{
  uint16_t adcValue = analogRead(BAT_VOLTAGE_PIN);
  float voltage = 3.3 * adcValue / 4095; // Convert to voltage (assuming 3.3V reference)
  return voltage*11.0; // Multiply by 11 to get actual voltage
}