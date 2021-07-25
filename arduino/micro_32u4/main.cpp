#include "SerialTransfer.h"
#include "I2CTransfer.h"
#include "Ultrasonic.h"
#include "Servo.h"

#undef DEBUG

#define SERVO_PIN 10
#define ESC_PIN 11
#define ENC_PIN 2

int neutral = 1500;
int minSpeed = neutral + 70;
int topSpeed = neutral + 150;
int minSpeedRev = neutral - 150;
int topSpeedRev = neutral - 250;

const int failsafeDelay = 300;
unsigned long lastInput = 0;
unsigned long lastUpdate = millis();
volatile unsigned int counter = 0;
unsigned int rps = 0;

Servo servo;
Servo ESC;

SerialTransfer transferSerial;
I2CTransfer transferI2C;

#define F_ECHO A0
#define F_TRIG A1 
#define FL_ECHO 8
#define FL_TRIG 9
#define FR_ECHO 6
#define FR_TRIG 7
#define TIMEOUT 3380 // 60 cm

Ultrasonic sonarF(F_TRIG, F_ECHO, TIMEOUT);
Ultrasonic sonarFL(FL_TRIG, FL_ECHO, TIMEOUT);
Ultrasonic sonarFR(FR_TRIG, FR_ECHO, TIMEOUT);

struct __attribute__((__packed__)) MotorStruct {
    int8_t relSpeed;
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

void count() {
    counter++;
}

void getRPM() {
  if (millis() - lastUpdate >= 1000) {
      rps = (counter / 1);  // divide by number of signals on wheel
      counter = 0;
      lastUpdate = millis();
      #ifdef DEBUG
          Serial.print("Speed: "); 
          Serial.print(rps, DEC);
          Serial.println(" rotations per seconds");
      #endif
  }
}

void recvDataI2C() {
    uint16_t recSize = 0;
    recSize = transferI2C.rxObj(mixedData, recSize);
    #ifdef DEBUG
        Serial.print(mixedData.distanceL);
        Serial.print(" | ");
        Serial.println(mixedData.distanceR);
    #endif
}

const functionPtr callbackArr[] = { recvDataI2C };

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
  
    #ifdef DEBUG
        Serial.begin(115200);
    #endif
    
    Serial1.begin(115200);
    transferSerial.begin(Serial1);
    
    Wire.begin(0);
    configST myConfig;
    myConfig.debug        = true;
    myConfig.callbacks    = callbackArr;
    myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
    transferI2C.begin(Wire, myConfig);
  
    servo.attach(SERVO_PIN);
    ESC.attach(ESC_PIN);

    attachInterrupt(digitalPinToInterrupt(2), count, FALLING);
  
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    ESC.writeMicroseconds(neutral);
    servo.write(90);
    delay(2000);
    digitalWrite(LED_BUILTIN, HIGH);
}

void recvDataSerial() {
    if(transferSerial.available()){
        uint16_t recSize = 0;
        recSize = transferSerial.rxObj(motors, recSize);
        lastInput = millis();
    }
}

void drive() {
    if (millis() - lastInput < failsafeDelay) {
        #ifdef DEBUG
            Serial.print(motors.relSpeed);
            Serial.print(" | ");
            Serial.print(motors.pos);
         #endif
    
        int relSpeed = motors.relSpeed;
        int pos = motors.pos;
        bool turbo = motors.turbo;

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
      
        #ifdef DEBUG
            Serial.println(absSpeed);
        #endif
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

    sonars.distF = distF;
    sonars.distFL = distFL;
    sonars.distFR = distFR;
}

void sendData() {
    uint16_t sendSize = 0;
    sendSize = transferSerial.txObj(sonars, sendSize);
    sendSize = transferSerial.txObj(mixedData, sendSize);
    transferSerial.sendData(sendSize);
}

void loop() {
    recvDataSerial();
    getRPM();
    drive();
    readDist();
    sendData();
}
