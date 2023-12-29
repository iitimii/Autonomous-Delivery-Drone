#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
 

RF24 radio(21, 5); 
const byte addresses[][6] = {"00001", "00002"};
uint8_t telemetry_loop_counter;
uint8_t error;

uint32_t loop_time;
uint32_t ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8;
float battery_voltage;
float angle_pitch, angle_roll;

 
struct TXData 
{
  uint8_t signature;
  uint32_t payload1;
  uint32_t payload2;
  int32_t payload3;
  int32_t payload4;
  float payload5;
  float payload6;
};

struct RXData 
{
  uint8_t signature;
  uint32_t payload1;
  uint32_t payload2;
  int32_t payload3;
  int32_t payload4;
  float payload5;
  float payload6;
};

TXData data_tx;
RXData data_rx;


// const char* ssid = "Galaxy A3003DB";
// const char* password = "enha6023";
const char* ssid = "tachys";
const char* password = "idontknowthepassword1704.";


WebSocketsServer webSocket = WebSocketsServer(80);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

        // send message to client
        webSocket.sendTXT(num, "Connected");
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);

      // echo data back to browser
      webSocket.sendTXT(num, payload);
      break;
  }
}

void sendDataToWebPage(uint8_t signature, uint32_t payload1, uint32_t payload2, int32_t payload3, int32_t payload4, float payload5, float payload6) {
  String message = String(signature) + "," + String(payload1) + "," + String(payload2) + "," + String(payload3) + "," + String(payload4) + "," + String(payload5) + "," + String(payload6);
  webSocket.broadcastTXT(message);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

    radio.begin();            
    radio.setChannel(120);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MIN); 
    radio.setAutoAck(false);
    radio.disableAckPayload();
    radio.disableDynamicPayloads();
    radio.openReadingPipe(0, addresses[0]); 
    radio.startListening();
}

void loop() {
  digitalWrite(LED_BUILTIN, WiFi.status() == WL_CONNECTED);

  while(!radio.available()){
    webSocket.loop();
  }
  radio.read(&data_rx, sizeof(RXData));

  sendDataToWebPage(data_rx.signature, data_rx.payload1, data_rx.payload2, data_rx.payload3, data_rx.payload4, data_rx.payload5, data_rx.payload6);

  webSocket.loop();
}


  // switch (data_rx.signature){

  //   case 'T':
  //   loop_time = data_rx.payload1;
  //   error = data_rx.payload2;
  //   ch1 = data_rx.payload3;
  //   ch2 = data_rx.payload4;
  //   battery_voltage = data_rx.payload5;
  //   angle_pitch = data_rx.payload6;
  //   break;

  //   case 'I':
  //   ch3 = data_rx.payload1;
  //   ch4 = data_rx.payload2;
  //   angle_roll = data_rx.payload6;
  //   break;

  //   default:
  //   break;
  // }
