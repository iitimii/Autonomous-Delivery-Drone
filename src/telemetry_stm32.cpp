// #include <Arduino.h>
// #include <SPI.h>
// #include <Wire.h>
// #include <nRF24L01.h>
// #include <RF24.h>
 
// RF24 radio(PB0, PA4); // CE, CSN on Blue Pill 
// const byte addresses[][6] = {"00001", "00002"};
 
// struct MyData 
// {
//   byte counter;
//   byte temperature;
//   byte humidity;
//   byte altitude;
//   byte pressure;
// };

// MyData data_rx;
// MyData data_tx;
 
// void setup() {
// radio.begin();            
// radio.setChannel(120);
// radio.setDataRate(RF24_250KBPS);
// radio.openWritingPipe(addresses[0]); 
// radio.openReadingPipe(1, addresses[1]);
// radio.setPALevel(RF24_PA_MIN); 
// }
 
// void loop()
// {
//   delay(5);
//   radio.stopListening();
//   radio.write(&data_tx, sizeof(data_tx));


//   delay(5);
//   radio.startListening();
//   if ( radio.available()) {
//     while (radio.available()) {
//       radio.read(&data_rx, sizeof(data_rx));
//     }
//   }
// }