
// #define ADC_RESOLUTION 12 // 12-bit resolution for STM32F401CCU6

// void setup() {
//   Serial.begin(115200);
//   pinMode(PB1, INPUT_ANALOG); // Configure PB1 as analog input
//   analogReadResolution(ADC_RESOLUTION); // Set ADC resolution to 12-bit
// }

// void loop() {
//   uint16_t adcValue = analogRead(PB1); // Read analog value from PA0
//   float voltage = 3.3 * adcValue / (pow(2, ADC_RESOLUTION) - 1); // Convert to voltage (assuming 3.3V reference)

//   Serial.print("ADC Value: ");
//   Serial.print(adcValue);
//   Serial.print(", Voltage: ");
//   Serial.print(voltage, 2); // Print voltage with 2 decimal places
//   Serial.println(" V");

//   delay(1000); // Wait for 1 second
// }