#include "SerialTransfer.h"
#include <PS4Controller.h>

SerialTransfer myTransfer;

struct Motors {
    int8_t relSpeed;
    uint8_t pos;
    bool turbo;
} motorStruct;

struct Sonars {
    uint8_t distF;
    uint8_t distFL;
    uint8_t distFR;
} sonarStruct;

char cmd[] = "move";
bool turbo = 0;
bool manual = 0;
bool lastCross = 0;
short lastSpeed = 0;

short distF;
short distFL;
short distFR;

long lastSend = millis();

void sendSpeed(int relSpeed = 0, int pos = 90, bool turbo = 0) {
    uint16_t sendSize = 0;
    motorStruct.relSpeed = relSpeed;
    motorStruct.pos = pos;
    motorStruct.turbo = turbo;
    sendSize = myTransfer.txObj(motorStruct, sendSize);
    sendSize = myTransfer.txObj(cmd, sendSize);
    myTransfer.sendData(sendSize);
}

void getSonar() {
    if(myTransfer.available()){
        uint16_t recSize = 0;
        recSize = myTransfer.rxObj(sonarStruct, recSize);
        recSize = myTransfer.rxObj(cmd, recSize);
        distF = sonarStruct.distF;
        distFL = sonarStruct.distFL;
        distFR = sonarStruct.distFR;
    }
}

void sendToJoystick() {
    short min;
    if (distF  <= distFL) {
        min = distF;
    }
    else {
        if (distFL <= distFR) {
            min = distFL;
        }
        else min = distFR;
    }

    short red = map(min, 0, 45, 255, 0);
    short green = 255 - red;

    PS4.setLed(red, green, 0);

    short rumbleL, rumbleR;
    if (distF <= distFL && distF <= distFR) {
        rumbleL = rumbleR = map(distF, 0, 45, 255, 0);
    }
    else {
        rumbleL = map(distFL, 0, 45, 255, 0);
        rumbleR = map(distFR, 0, 45, 255, 0);
    }
    
    PS4.setRumble(rumbleL, rumbleR);
    PS4.sendToController();
}

bool operator==(const Sonars lhs, const Sonars rhs) {
    if ((lhs.distF - rhs.distF < 3) && (lhs.distF - rhs.distF > -3)
        && (lhs.distFL - rhs.distFL < 3) && (lhs.distFL - rhs.distFL > -3)
        && (lhs.distFR - rhs.distFR < 3) && (lhs.distFR - rhs.distFR > -3))
            return 1;
    else return 0;
}

bool operator!=(const Sonars lhs, const Sonars rhs) {
    return !(lhs == rhs);
}

void setup()
{
  Serial.begin(115200);
  PS4.begin("00:de:ad:be:ef:00");
  Serial2.begin(115200);
  myTransfer.begin(Serial2);
}


void loop()
{
    int mapX, mapY = 0;

    if (PS4.isConnected()) {
        //turbo
        if (PS4.Triangle()) {
            turbo = 1;
        }
        else {
            turbo = 0;
        }
        // forward
        if (PS4.R2Value() > 10 && PS4.L2Value() < 10) {
          mapY = map(PS4.R2Value(), 0, 255, 0, 100);
        }
        // backward
        else if (PS4.L2Value() > 10 && PS4.R2Value() < 10) {
          mapY = map(PS4.L2Value(), 0, 255, 0, -100);
        }
        // brake
        else {
          mapY = 0;
        }
      
        getSonar();
      
        if ((millis() - lastSend) > 30) {
            sendToJoystick();
            lastSend = millis();
        }
      
        Serial.print("Sonar front: ");
        Serial.println(distF);
        Serial.print("Sonar front left: ");
        Serial.println(distFL);
        Serial.print("Sonar front right: ");
        Serial.println(distFR);
      
        bool cross = PS4.Cross();
        if (cross != lastCross) {
            if (cross) {
                manual = !manual;
            }
        }
        lastCross = cross;
      
        if (manual) {
            mapX = map(PS4.LStickX(), -128, 127, 0, 180);
        }
        else {
            if (distF >= 45 && distFL >= 45 & distFR >= 45) {
                mapX = 90;
            }
            else if (distF <= 45 && (distFL < 45 | distFR < 45) && (distFL > 10 && distFR > 10))  {
                if (distFL < distFR) {
                    if (mapY >= 0) {
                        mapX = 90 + 90;
                    }
                    else {
                        mapX = 90 - 90;
                        if (lastSpeed > 0) {
                            sendSpeed();
                            delay(100);
                        }
                    }
                }
                else {
                    if (mapY >= 0) {
                        mapX = 90 - 90;
                    }
                    else {
                        mapX = 90 + 90;
                        if (lastSpeed > 0) {
                            sendSpeed();
                            delay(100);
                        }
                    }
                }
            }
            else if (distF <= 45 && (distFL <= 10 | distFR <= 10)) {
                sendSpeed();
                delay(200);
                Sonars values1 = sonarStruct;
                getSonar();
                if (values1 != sonarStruct) {
                    Sonars values2 = sonarStruct;
                    delay(100);
                    getSonar();
                    if (sonarStruct != values2) {
                        sonarStruct.distF = (sonarStruct.distF + values1.distF + values2.distF) / 3;
                        sonarStruct.distFL = (sonarStruct.distFL + values1.distFL + values2.distFL) / 3;
                        sonarStruct.distFR = (sonarStruct.distFR + values1.distFR + values2.distFR) / 3;
                    }
                }
          
                if (distF <= 45 && distFL < 10 | distFR < 10) {
                    mapY = -60;
                    mapX = 90;
                }
            }
        }
      
        Serial.print("Angle: ");
        Serial.println(mapX);
      
        sendSpeed(mapY, mapX, turbo);
        lastSpeed = mapY;
    }
    else {
        sendSpeed();
    }
}
