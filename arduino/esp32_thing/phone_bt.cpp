#include "SerialTransfer.h"

#define BLYNK_USE_DIRECT_CONNECT
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

char auth[] = "r70VT9kv6x1BQJ_iCe7zRaqItWJf_OBS";

SerialTransfer myTransfer;

struct __attribute__((__packed__)) MotorStruct {
    int8_t speed;
    uint8_t pos;
    bool turbo;
} motors;

struct __attribute__((__packed__)) SonarStruct {
    uint8_t distF;
    uint8_t distFL;
    uint8_t distFR;
} sonars;

struct __attribute__((__packed__)) MixedStruct {
    uint8_t distanceL;
    uint8_t distanceR;
    double lat;
    double lng;
    double speed;
    char dir;
} mixedData;

struct OldData {
    struct SonarStruct sonars;
    struct MixedStruct mixedData;
    struct MotorStruct motors;
    bool manual;
} old {{60, 60, 60}, {40, 40, 0, 0, 0, 'L'}, {0, 90, 0}, 1};


bool brake = 1;
bool manual = 1;
bool fullAuto = 0;
int maxSpeed = 50;

short distF;
short distFL;
short distFR;
short distL;
short distR;

long distLFL;
long distRFR;

int mapX = 90;
int mapY = 0;

void sendSpeed(int speed = 0, int pos = 90, bool turbo = 0) {
    uint16_t sendSize = 0;
    motors.speed = speed;
    motors.pos = pos;
    motors.turbo = turbo;
    sendSize = myTransfer.txObj(motors, sendSize);
    myTransfer.sendData(sendSize);
}

void recvData() {
    if (myTransfer.available()){
        uint16_t recSize = 0;
        recSize = myTransfer.rxObj(sonars, recSize);
        recSize = myTransfer.rxObj(mixedData, recSize);
        distF = sonars.distF;
        distFL = sonars.distFL;
        distFR = sonars.distFR;
        distL = mixedData.distanceL;
        distR = mixedData.distanceR;
    }
}

bool operator==(const SonarStruct lhs, const SonarStruct rhs) {
    if ((lhs.distF - rhs.distF < 2) && (lhs.distF - rhs.distF > -2)
        && (lhs.distFL - rhs.distFL < 2) && (lhs.distFL - rhs.distFL > -2)
        && (lhs.distFR - rhs.distFR < 2) && (lhs.distFR - rhs.distFR > -2))
            return 1;
    else return 0;
}

bool operator!=(const SonarStruct lhs, const SonarStruct rhs) {
    return !(lhs == rhs);
}

SonarStruct operator/(SonarStruct lhs, const int rhs) {
    lhs.distF = lhs.distF / rhs;
    lhs.distFL = lhs.distFL / rhs;
    lhs.distFR = lhs.distFR / rhs;
    return lhs;
}

void calc() {
    if ((distFL <= distFR) && (distL <= distR)) {
        if (mapY >= 0) {
            mapX = 90 + 90;
        }
        else {
            mapX = 90 - 90;
            if (fullAuto) {
                if (old.motors.speed > 0) {
                    sendSpeed();
                    delay(100);
                }
            }
        }
    }
    else if ((distFL >= distFR) && (distL >= distR)) {
        if (mapY >= 0) {
            mapX = 90 - 90;
        }
        else {
            mapX = 90 + 90;
            if (fullAuto) {
                if (old.motors.speed > 0) {
                    sendSpeed();
                    delay(100);
                }
            }
        }
    }
    else {
        distLFL = 5 * distL + 4 * distFL;
        distRFR = 5 * distR + 4 * distFR;
        if (distLFL >= distRFR) {
            if (mapY >= 0) {
                mapX = 90 - 90;
            }
            else {
                mapX = 90 + 90;
                if (fullAuto) {
                    if (old.motors.speed > 0) {
                        sendSpeed();
                        delay(100);
                    }
                }
            }
        }
        else {
            if (mapY <= 0) {
                mapX = 90 + 90;
            }
            else {
                mapX = 90 - 90;
                if (fullAuto) {
                    if (old.motors.speed > 0) {
                        sendSpeed();
                        delay(100);
                    }
                }
            }
        }
    }
}

void calcDir() {
    if (sonars != old.sonars || ((mixedData.distanceL != old.mixedData.distanceL) || (mixedData.distanceR != old.mixedData.distanceR))) {
        if (distF >= 60 && (distFL >= 60 && distFR >= 60) && (distL >= 40 && distR >= 40)) {
            mapX = 90;
        }
        else if ((distF <= 60) && (((distFL < 60 || distFR < 60) && (distFL > 40 && distFR > 40)) || ((distL < 40 || distR < 40) && (distR > 25 && distL > 25)))) {
            calc();
        }
        else if (((distFL <= 40 || distFR <= 40) && (distFL > 10 && distFR > 10)) || ((distL <= 25 || distR <= 25) && (distL > 5 && distR > 5))) {
            calc();
        }
        else if ((distFL <= 10) || (distFR <= 10) || (distL <= 5) || (distR <= 5)) {
            if (fullAuto) {
                sendSpeed();
                delay(200);
                SonarStruct values1 = sonars;
                recvData();
                if (values1 != sonars) {
                    SonarStruct values2 = sonars;
                    delay(100);
                    recvData();
                    if (sonars != values2) {
                        sonars.distF = (sonars.distF + values1.distF + values2.distF) / 3;
                        sonars.distFL = (sonars.distFL + values1.distFL + values2.distFL) / 3;
                        sonars.distFR = (sonars.distFR + values1.distFR + values2.distFR) / 3;
                    }
                }

                if ((distFL < 10) || (distFR < 10) || (distL < 5) || (distR < 5)) {
                    mapY = -60;
                    mapX = 90;
                }
            }
        }
    }
    else {
        mapX = old.motors.pos;
    }
}

void printDebug() {
    #ifdef DEBUG
        Serial.print("Sonar front: ");
        Serial.println(distF);
        Serial.print("Sonar front left: ");
        Serial.println(distFL);
        Serial.print("Sonar front right: ");
        Serial.println(distFR);
        Serial.print("Sonar left: ");
        Serial.println(distL);
        Serial.print("Sonar right: ");
        Serial.println(distR);
        Serial.println();
        Serial.print("Steering angle: ");
        Serial.println(mapX);
        Serial.print("Throttle: ");
        Serial.println(mapY);
        Serial.println();
    #endif
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


void loop() {
  Blynk.run();
  recvData();
  if (!manual) {
    calcDir();
  }
  if (!brake) {
    if (!manual && mapY == 0) {
        sendSpeed(mapY, old.motors.pos);
    }
    else {
        sendSpeed(mapY, mapX);
    }
  }
  else {
    sendSpeed();
  }
  old = {sonars, mixedData, motors, manual};
  printDebug();
}

BLYNK_WRITE(V0) {
  brake = !param.asInt();
}

BLYNK_WRITE(V1) {
  int posX = param[0].asInt();
  int posY = param[1].asInt();
  mapX = map(posX, -255, 255, 0, 180);
  mapY = map(posY, -255, 255, maxSpeed*-1, maxSpeed);
  if (posY == 0) mapY = 0;
}

BLYNK_WRITE(V2) {
  maxSpeed = param.asInt();
}

BLYNK_WRITE(V3) {
  manual = !param.asInt();
}