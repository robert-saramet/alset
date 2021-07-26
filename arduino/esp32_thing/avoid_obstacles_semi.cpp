#include "SerialTransfer.h"
#include <PS4Controller.h>

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

bool turbo = 0;
bool manual = 0;
bool lastCross = 0;
bool fullAuto = 0;

short distF;
short distFL;
short distFR;
short distL;
short distR;

long distLFL;
long distRFR;

struct OldData {
    struct SonarStruct sonars;
    struct MixedStruct mixedData;
    struct MotorStruct motors;
    bool manual;
} old {{60, 60, 60}, {40, 40, 0, 0, 0, 'L'}, {0, 90, 0}, 1};

long lastSend = millis();

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

void sendToJoystick() {
    short min;
    if (distF  <= distFL) {
        if (distF <= distFR) {
            min = distF;
        }
        else {
            min = distFR;
        }
    }
    else {
        if (distFL <= distFR) {
            min = distFL;
        }
        else min = distFR;
    }

    if (min > 45) {
        min = 45;
    }
    short red = map(min, 0, 45, 255, 0);
    short green = 255 - red;

    PS4.setLed(red, green, 0);

    short rumbleL, rumbleR;
    if (distF <= distFL && distF <= distFR) {
        rumbleL = rumbleR = map(distF, 0, 60, 255, 0);
    }
    else {
        rumbleL = map(distFL, 0, 60, 255, 0);
        rumbleR = map(distFR, 0, 60, 255, 0);
    }
    
    PS4.setRumble(rumbleL, rumbleR);
    PS4.sendToController();
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
      
        recvData();
      
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
            if (sonars != old.sonars || ((mixedData.distanceL != old.mixedData.distanceL) || (mixedData.distanceR != old.mixedData.distanceR))) {
                if (distF >= 60 && (distFL >= 60 && distFR >= 60) && (distL >= 40 && distR >= 40)) {
                    mapX = 90;
                }
                else if (distF <= 60 && ((distFL < 60 | distFR < 60) && (distFL > 40 && distFR > 40)) || ((distL < 40 && distR < 40) && (distR > 25 && distL > 25)))  {
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
                else if ((distFL <= 40 || distFR <= 40) && (distFL > 10 && distFR > 10) || ((distL <= 25 || distR <= 25) && (distL > 5 && distR > 5)))  {
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
                        if (distLFL > distRFR) {
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
      
        Serial.print("Angle: ");
        Serial.println(mapX);
      
        sendSpeed(mapY, mapX, turbo);
        old = {sonars, mixedData, motors, manual};
    }
    else {
        sendSpeed();
    }
}
