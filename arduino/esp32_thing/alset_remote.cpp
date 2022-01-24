#include "SerialTransfer.h"
#include <esp_now.h>
#include <WiFi.h>

#define DEBUG

struct RemoteData {
    int relSpeed, pos;
    bool turbo, autoPilot;
} remDat = {0, 90, 0, 0};

struct __attribute__((__packed__)) MotorStruct {
    int8_t relSpeed;
    uint8_t pos;
    bool turbo;
} motors = {0, 90, 0};

long lastRecv = millis();

SerialTransfer serialTransfer;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&remDat, incomingData, sizeof(remDat));
    #ifdef DEBUG
        Serial.print(F("Bytes received: "));
        Serial.println(len);
        Serial.print(F("Time since last input: "));
        Serial.print(millis() - lastRecv);
        Serial.println(F("ms"));
    #endif
    lastRecv = millis();
    sendSpeed();
}

void sendSpeed() {
    uint16_t sendSize = 0;
    if (millis() - lastRecv > 200)
        remDat = {0, 90, 0, 0};
    motors.relSpeed = remDat.relSpeed;
    motors.pos = remDat.pos;
    motors.turbo = remDat.turbo;
    sendSize = serialTransfer.txObj(motors, sendSize);
    serialTransfer.sendData(sendSize);
}

void initComms() {
    Serial.begin(115200);
    Serial2.begin(115200);
    serialTransfer.begin(Serial2);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        #ifdef DEBUG
            Serial.println(F("Error initializing ESP-NOW"));
        #endif
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void setup() {
    initComms();
    delay(200);
}

void loop() {
    
}
