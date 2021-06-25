#include "SerialTransfer.h"
#include "Servo.h"

#define SERVO_PIN 10
#define ESC_PIN 11

const int neutral = 1500;
const int minSpeed = neutral + 70;
const int topSpeed = neutral + 150;
const int minSpeedRev = neutral - 150;
const int topSpeedRev = neutral - 250;

const int failsafeDelay = 300;
long lastInput = 0;

Servo servo;
Servo ESC;

SerialTransfer myTransfer;

struct STRUCT {
    int8_t relSpeed;
    uint8_t pos;
} testStruct;

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


void loop() {
    if(myTransfer.available()){
        uint16_t recSize = 0;
        recSize = myTransfer.rxObj(testStruct, recSize);
        recSize = myTransfer.rxObj(cmd, recSize);
        lastInput = millis();
    }
    
    if (millis() - lastInput < failsafeDelay) {
        Serial.print(testStruct.relSpeed);
        Serial.print(" | ");
        Serial.print(testStruct.pos);
        Serial.print(" | ");
        Serial.println(cmd);
    
        int relSpeed = testStruct.relSpeed;
        int pos = testStruct.pos;
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
