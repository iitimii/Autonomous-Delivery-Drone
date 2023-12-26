// #include <Arduino.h>
// #include <WiFiMulti.h>
// #include <WebSocketsClient.h>
// #include <ArduinoJson.h>
// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <RF24_config.h>
 

// RF24 radio(21, 5); 
// const byte addresses[][6] = {"00001", "00002"};
// uint8_t telemetry_loop_counter;
// uint8_t error;

// uint32_t loop_time;
// uint32_t ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8;
// float battery_voltage;
// float angle_pitch, angle_roll;

 
// struct TXData 
// {
//   uint8_t signature;
//   uint32_t payload1;
//   uint32_t payload2;
//   int32_t payload3;
//   int32_t payload4;
//   float payload5;
//   float payload6;
// };

// struct RXData 
// {
//   uint8_t signature;
//   uint32_t payload1;
//   uint32_t payload2;
//   int32_t payload3;
//   int32_t payload4;
//   float payload5;
//   float payload6;
// };

// TXData data_tx;
// RXData data_rx;


// #define WIFI_SSID "tachys"
// #define WIFI_PASSWORD "idontknowthepassword1704."

// #define WS_HOST "ws"
// #define WS_PORT 443
// #define WS_URL ""

// // ws://localhost:8080

// #define JSON_DOC_SIZE 2048
// #define MSG_SIZE 256

// WiFiMulti wifiMulti;
// WebSocketsClient wsClient;

// void sendErrorMessage(const char * error){
//   char msg[MSG_SIZE];
//   sprintf(msg, "{\"action\":\"msg\",\"type\":\"error\",\"body\":\"%s\"}", error);
//   wsClient.sendTXT(msg);
// }

// void sendOkayMessage(){
//   wsClient.sendTXT("{\"action\":\"msg\",\"type\":\"status\",\"body\":\"ok\"}");
// }

// uint8_t toMode(const char *val){
//   if (strcmp(val, "output ") == 0){
//     return OUTPUT;
//   }
//   if (strcmp(val, "input_pullup") == 0){
//     return INPUT_PULLUP;
//   }
//   return INPUT;
// }

// void handleMessage(uint8_t * payload){
//   StaticJsonDocument<JSON_DOC_SIZE> doc;

//    DeserializationError error = deserializeJson(doc, payload);

//   // Test if parsing succeeds.
//   if (error) {
//     Serial.print(F("deserializeJson() failed: "));
//     Serial.println(error.f_str());
//     sendErrorMessage(error.c_str());
//     return;
//   }

//   if (!doc["type"].is<const char *>()) {
//     sendErrorMessage("Invalid message");
//     return;
//   } 

//   if (strcmp(doc["type"], "cmd") == 0) {
//     if (!doc["body"].is<JsonObject>()){
//       sendErrorMessage("Invalid Command Body");
//       return;
//     }
//     if (strcmp(doc["body"]["type"], "pinMode") == 0){
//        pinMode(doc["body"]["pin"], toMode(doc["body"]["mode"]));
//        sendOkayMessage();
//        return;
//     }
    
//     if (strcmp(doc["body"]["type"], "digitalWrite") == 0){
//         digitalWrite(doc["body"]["pin"], strcmp(doc["body"]["value"], "HIGH") == 0 ? HIGH : LOW);
//         sendOkayMessage();
//         return;
// }

//     if (strcmp(doc["body"]["type"], "digitalRead") == 0){
//        auto value = digitalRead(doc["body"]["pin"]);

//        char msg[MSG_SIZE];
//       sprintf(msg, "{\"action\":\"msg\",\"type\":\"output\",\"body\":%d}", error);
//       wsClient.sendTXT(msg);
//        return;
//     }
//     sendErrorMessage("Unsupported Command Type");
//     return;
//   }
//    sendErrorMessage("Unsupported message");
//    return;
// }

// void onWSEvent(WStype_t type, uint8_t * payload, size_t length){
//   switch(type){
//     // WStype_ERROR,
//     case WStype_DISCONNECTED:
//       Serial.println("Disconnected");
//       break;
//     case WStype_CONNECTED:
//       Serial.println("Connected");
//       break;
//     case WStype_TEXT:
//       Serial.printf("WS Message: %s\n", payload);
//       handleMessage(payload);

//       break;
//     // WStype_BIN,
//     // WStype_FRAGMENT_TEXT_START,
//     // WStype_FRAGMENT_BIN_START,
//     // WStype_FRAGMENT,
//     // WStype_FRAGMENT_FIN,
//     // WStype_PING,
//     // WStype_PONG,

//   }

// }

// void setup() {
//   Serial.begin(921600);
//   pinMode(LED_BUILTIN, OUTPUT);
//   radio.begin();            
//   radio.setChannel(120);
//   radio.setDataRate(RF24_250KBPS);
//   radio.setPALevel(RF24_PA_MIN); 
//   radio.setAutoAck(false);
//   radio.disableCRC();
//   radio.disableAckPayload();
//   radio.disableDynamicPayloads();
//   radio.openReadingPipe(0, addresses[0]);
//   radio.startListening();

//   wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

//   while (wifiMulti.run() != WL_CONNECTED) {
//     delay(100);
//   }

//   Serial.println("Connected");

//   wsClient.beginSSL(WS_HOST, WS_PORT, WS_URL, "", "wss");
//   wsClient.onEvent(onWSEvent);
  
// }

// void loop() {
//   digitalWrite(LED_BUILTIN, WiFi.status() == WL_CONNECTED);
//   wsClient.loop();

//   while(!radio.available());
//   radio.read(&data_rx, sizeof(RXData));
//   switch (data_rx.signature){

//     case 'T':
//     loop_time = data_rx.payload1;
//     error = data_rx.payload2;
//     ch1 = data_rx.payload3;
//     ch2 = data_rx.payload4;
//     battery_voltage = data_rx.payload5;
//     angle_pitch = data_rx.payload6;
//     break;

//     case 'I':
//     ch3 = data_rx.payload1;
//     ch4 = data_rx.payload2;
//     angle_roll = data_rx.payload6;
//     break;

//     case 'E':
//     break;

//     default:
//     break;
//   }


// }






#include <WiFi.h>
#include <WebSocketsServer.h>

// Replace with your network credentials
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
}

void loop() {
  digitalWrite(LED_BUILTIN, WiFi.status() == WL_CONNECTED);
  webSocket.loop();
  webSocket.sendTXT(0, "Hello world");
}

