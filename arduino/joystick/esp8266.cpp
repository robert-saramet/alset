// Receiver

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SerialTransfer.h>

// Must match the transmitter structure
typedef struct struct_message {
   int16_t posX1;
   int16_t posY1;
   bool locked1;
   int16_t posX2;
   int16_t posY2;
   bool locked2;
} struct_message;

// Create instance of struct_message called myMessage
struct_message myMessage;

// Must match the receiver structure
struct STRUCT {
  int16_t pos_x1;
  int16_t pos_y1;
  bool sw1;
  int16_t pos_x2;
  int16_t pos_y2;
  bool sw2;
} payload;

SerialTransfer myTransfer;

// Callback when data is received
void onDataReceived(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
   // We don't use mac to verify the sender
   // Transform the incomingData into our message structure
  memcpy(&myMessage, incomingData, sizeof(myMessage));
}

void setup() {
  Serial.begin(4800);
  myTransfer.begin(Serial);
  WiFi.disconnect();
  ESP.eraseConfig();
 
  // Wifi Station Mode
  WiFi.mode(WIFI_STA);
  // Get Mac Address
  
  // Initializing the ESP-NOW
  if (esp_now_init() != 0) {
    //Serial.println("Problem during ESP-NOW init");
    return;
  }
  
  //esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  // Register the receiver callback function
  esp_now_register_recv_cb(onDataReceived);
}
void loop() {
  payload.pos_x1 = myMessage.posX1;
  payload.pos_y1 = myMessage.posY1;
  payload.sw1 = myMessage.locked1;
  payload.pos_x2 = myMessage.posX2;
  payload.pos_y2 = myMessage.posY2;
  payload.sw2 = myMessage.locked2;

  myTransfer.sendDatum(payload);
}
