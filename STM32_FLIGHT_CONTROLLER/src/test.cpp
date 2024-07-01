// #include <Arduino.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <RF24_config.h>

// uint32_t loop_time, loop_time_actual;
// struct RadioData
// {
//     uint8_t signature;
//     int16_t payload1;
//     int16_t payload2;
//     int16_t payload3;
//     float payload4;
//     float payload5;
//     float payload6;
//     float payload7;
//     float payload8;
//     float payload9;
// };

// int irq_pin = PA1;
// RF24 radio(PB0, PA4);
// const byte addresses[][6] = {"00001", "00002"};

// RadioData data_tx;
// RadioData data_rx;

// volatile bool dataReceived = false;

// void interrupt()
// {
//     dataReceived = true;
// }

// void setup()
// {
//     radio.begin();
//     radio.setChannel(120);
//     radio.setDataRate(RF24_250KBPS);
//     radio.setPALevel(RF24_PA_MIN);
//     radio.setAutoAck(false);
//     radio.disableDynamicPayloads();
//     // radio.setRetries(0, 2);
//     radio.maskIRQ(1, 1, 0);
//     attachInterrupt(digitalPinToInterrupt(irq_pin), interrupt, FALLING);
//     radio.openWritingPipe(addresses[0]);
//     radio.openReadingPipe(0, addresses[1]);
//     radio.startListening();
// }

// void loop()
// {
//     loop_time = micros();
//     loop_time_actual = micros();
//     data_tx.signature = 'T';
//     data_tx.payload9 = 889.345;

//     radio.stopListening();
//     radio.write(&data_tx, sizeof(data_tx));
//     radio.startListening();

//     data_tx.payload1 = micros() - loop_time_actual;

//     while (micros() - loop_time < 4000)
//         ;
// }
