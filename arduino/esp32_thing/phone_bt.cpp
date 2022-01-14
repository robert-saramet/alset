#include "SerialTransfer.h"

#define BLYNK_USE_DIRECT_CONNECT
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

char auth[] = "r70VT9kv6x1BQJ_iCe7zRaqItWJf_OBS";

SerialTransfer myTransfer;

struct STRUCT {
  int8_t relSpeed;
  uint8_t pos;
} testStruct;

char cmd[] = "move";

bool brake = 1;

int maxSpeed = 50;

void sendSpeed(int relSpeed = 0, int pos = 0) {
    uint16_t sendSize = 0;
    testStruct.relSpeed = relSpeed;
    testStruct.pos = pos;
    sendSize = myTransfer.txObj(testStruct, sendSize);
    sendSize = myTransfer.txObj(cmd, sendSize);
    myTransfer.sendData(sendSize);
}

void setup()
{
  Serial.begin(115200);
  Blynk.setDeviceName("Alset");
  Blynk.begin(auth);
  delay(300);
  Serial2.begin(115200);
  myTransfer.begin(Serial2);
}


void loop()
{
  Blynk.run();
}

BLYNK_WRITE(V0)
{
  bool pinValue = param.asInt(); 
  if (pinValue) {
    brake = 0;
  }
  else {
    brake = 1;
  }
}

BLYNK_WRITE(V1) {
  if (!brake) {
    int posX = param[0].asInt();
    int posY = param[1].asInt();
    int mapX = map(posX, -255, 255, 0, 180);
    int mapY = map(posY, -255, 255, maxSpeed*-1, maxSpeed);
    sendSpeed(mapY, mapX);
  }
  else {
    sendSpeed(0, 90);
  }
}


BLYNK_WRITE(V2) {
  maxSpeed = param.asInt();
}
