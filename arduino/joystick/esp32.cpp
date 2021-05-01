// Transmitter

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Joystick 1 (robot) pins
#define VRx1 32
#define VRy1 33
#define SW1 22

// Joystick 1 values
int xPosition1 = 0;
int yPosition1 = 0;
bool SW_state1 = 0;
int mapX1 = 0;
int mapY1 = 0;

// Joystick 2 (laser) pins
#define VRx2 34
#define VRy2 35
#define SW2 21

// Joystick 2 values
int xPosition2 = 0;
int yPosition2 = 0;
bool SW_state2 = 0;
int mapX2 = 0;
int mapY2 = 0;

// Mac address of the slave
uint8_t broadcastAddress[] = {0x48, 0x3F, 0xDA, 0x50, 0x02, 0xC6};

// Must match the receiver structure
typedef struct struct_message {
   int16_t posX1;
   int16_t posY1;
   bool locked1;
   int16_t posX2;
   int16_t posY2;
   bool locked2;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, register for
  // 'Send CB' to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  // Initialize joystick 1
  pinMode(VRx1, INPUT);
  pinMode(VRy1, INPUT);
  pinMode(SW1, INPUT_PULLUP);

  // Initialize joystick 2
  pinMode(VRx2, INPUT);
  pinMode(VRy2, INPUT);
  pinMode(SW2, INPUT_PULLUP); 
}
 
void loop() {
  // Read joystick 1 values
  analogReadResolution(9);
  xPosition1 = analogRead(VRx1);
  yPosition1 = analogRead(VRy1);
  SW_state1 = !digitalRead(SW1);
  mapX1 = map(xPosition1, 0, 511, -255, 255);
  mapY1 = map(yPosition1, 0, 511, -255, 255);

  // Read joystick 2 values
  xPosition2 = analogRead(VRx2);
  yPosition2 = analogRead(VRy2);
  SW_state2 = !digitalRead(SW2);
  mapX2 = map(xPosition2, 0, 511, -255, 255);
  mapY2 = map(yPosition2, 0, 511, -255, 255);
  
  // Set values to send
  myData.posX1 = mapX1;
  myData.posY1 = mapY1;
  myData.locked1 = SW_state1;
  myData.posX2 = mapX2;
  myData.posY2 = mapY2;
  myData.locked2 = SW_state2;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  // Avoid overheating by spamming messages
  delay(100);
}
