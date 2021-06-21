#include "SerialTransfer.h"
#include "Servo.h"

#define SERVO_PIN 10
#define ESC_PIN 11

const int neutral = 1500;
const int minSpeed = 60;
const int topSpeed = 120;

Servo servo;
Servo ESC;

SerialTransfer myTransfer;

struct STRUCT {
  int8_t relSpeed;
  uint8_t pos;
} testStruct;

char arr[6];


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
  delay(4000);
  digitalWrite(LED_BUILTIN, HIGH);
}


void loop() {
  if(myTransfer.available()) {
    uint16_t recSize = 0;

    recSize = myTransfer.rxObj(testStruct, recSize);
    Serial.print(testStruct.relSpeed);
    Serial.print(" | ");
    Serial.print(testStruct.pos);
    Serial.print(" | ");

    recSize = myTransfer.rxObj(arr, recSize);
    Serial.println(arr);

    int relSpeed = testStruct.relSpeed;
    int pos = testStruct.pos;

    int absSpeed;
  if (relSpeed > 0 && relSpeed <= 100) {
    absSpeed = map(relSpeed, 1, 100, neutral + minSpeed, neutral + topSpeed);
  }
  else if (relSpeed < 0 && relSpeed >= -100) {
    absSpeed = map(relSpeed, -1, -100, neutral - minSpeed, neutral - topSpeed);
    ESC.writeMicroseconds(neutral);
    delay(50);
  }
  else {
    absSpeed = neutral;
  }

  Serial.println(absSpeed);
  ESC.writeMicroseconds(absSpeed);
  servo.write(pos);
  }
}
