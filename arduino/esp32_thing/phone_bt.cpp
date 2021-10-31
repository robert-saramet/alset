#include "SerialTransfer.h"
#include "BluetoothSerial.h"

#include <BlynkSimpleSerialBLE.h>

char auth[] = "B3FFB419A40961239E202763E131A519C92915D048A32690A9E71F06A5E85CEB72ED63045F2791BAD5EF60CFD990AE85";

SerialTransfer myTransfer;
BluetoothSerial SerialBT;

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
  SerialBT.begin("Alset v2");
  Blynk.begin(SerialBT, auth);
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
