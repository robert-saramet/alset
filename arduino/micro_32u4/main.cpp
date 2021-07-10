#include "SerialTransfer.h"
#include "Ultrasonic.h"
#include "Servo.h"

#undef DEBUG

#define SERVO_PIN 10
#define ESC_PIN 11

int neutral = 1500;
int minSpeed = neutral + 70;
int topSpeed = neutral + 150;
int minSpeedRev = neutral - 150;
int topSpeedRev = neutral - 250;

const int failsafeDelay = 300;
long lastInput = 0;

Servo servo;
Servo ESC;

SerialTransfer myTransfer;

#define F_ECHO A0
#define F_TRIG A1 
#define FL_ECHO 8
#define FL_TRIG 9
#define FR_ECHO 6
#define FR_TRIG 7
#define TIMEOUT 6000

Ultrasonic sonarF(F_TRIG, F_ECHO, TIMEOUT);
Ultrasonic sonarFL(FL_TRIG, FL_ECHO, TIMEOUT);
Ultrasonic sonarFR(FR_TRIG, FR_ECHO, TIMEOUT);

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

char cmd[6];



void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
  
    Serial.begin(115200);
    Serial1.begin(115200);
    myTransfer.begin(Serial1);
  
    servo.attach(SERVO_PIN);
    ESC.attach(ESC_PIN);
  
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    ESC.writeMicroseconds(neutral);
    servo.write(90);
    delay(4000);
    digitalWrite(LED_BUILTIN, HIGH);
}

void recvData() {
    if(myTransfer.available()){
        uint16_t recSize = 0;
        recSize = myTransfer.rxObj(motorStruct, recSize);
        recSize = myTransfer.rxObj(cmd, recSize);
        lastInput = millis();
    }
}

void drive() {
    if (millis() - lastInput < failsafeDelay) {
        #ifdef DEBUG
            Serial.print(motorStruct.relSpeed);
            Serial.print(" | ");
            Serial.print(motorStruct.pos);
            Serial.print(" | ");
            Serial.println(cmd);
         #endif
    
        int relSpeed = motorStruct.relSpeed;
        int pos = motorStruct.pos;
        bool turbo = motorStruct.turbo;

        if (turbo) {
            minSpeed = neutral + 100;
            topSpeed = neutral + 200;
            minSpeedRev = neutral - 200;
            topSpeedRev = neutral - 300;
        }
        else {
            neutral = 1500;
            minSpeed = neutral + 70;
            topSpeed = neutral + 150;
            minSpeedRev = neutral - 150;
            topSpeedRev = neutral - 250;
        }
        
        int absSpeed;
        bool lastDir;
        
        if (relSpeed > 0 && relSpeed <= 100) {
            absSpeed = map(relSpeed, 1, 100, minSpeed, topSpeed);
            lastDir = 1;
        }
        else if (relSpeed < 0 && relSpeed >= -100) {
            absSpeed = map(relSpeed, -1, -100, minSpeedRev, topSpeedRev);
            long revDelay = millis();
            if (lastDir) {
                while (millis() - revDelay < 100) {
                    ESC.writeMicroseconds(neutral);
                }
            }
            ESC.writeMicroseconds(absSpeed);
            lastDir = 0;
        }
        else {
            absSpeed = neutral;
            lastDir = 0;
        }
      
        int mapPos = map(pos, 0, 180, 180, 0);
      
        Serial.println(absSpeed);
        ESC.writeMicroseconds(absSpeed);
        servo.write(mapPos);
    }
    else {
        ESC.writeMicroseconds(neutral);
    }
}

void readDist() {
    short distF = sonarF.read();
    short distFL = sonarFL.read();
    short distFR = sonarFR.read();

    #ifdef DEBUG
        Serial.print("Sensor front: ");
        Serial.print(distF);
        Serial.println(" cm");
      
        Serial.print("Sensor front left: ");
        Serial.print(distFL);
        Serial.println(" cm");
      
        Serial.print("Sensor front right: ");
        Serial.print(distFR);
        Serial.println(" cm");
    #endif

    sonarStruct.distF = distF;
    sonarStruct.distFL = distFL;
    sonarStruct.distFR = distFR;
}

void sendData() {
    uint16_t sendSize = 0;
    sendSize = myTransfer.txObj(sonarStruct, sendSize);
    myTransfer.sendData(sendSize);
}

void loop() {
    recvData();
    drive();
    readDist();
    sendData();
}
