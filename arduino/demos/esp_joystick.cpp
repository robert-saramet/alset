#include <Arduino.h>
#include <L298NX2.h>
//#include <Servo.h>
#include <SerialTransfer.h>

#define motorA1 2
#define motorA2 3
#define motorB1 4
#define motorB2 5
#define motorA_EN 6
#define motorB_EN 7
#define STBY 9

struct STRUCT {
  int16_t pos_x1;
  int16_t pos_y1;
  bool sw1;
  int16_t pos_x2;
  int16_t pos_y2;
  bool sw2;
} payload;

int mapX1, mapY1;
bool btn1 = 0;

int mapX2, mapY2;
bool btn2 = 0;

bool brake = 1;
bool pwr = 1;
int pos = 130;

SerialTransfer myTransfer;
L298NX2 robot(motorB_EN, motorB1, motorB2, motorA_EN, motorA1, motorA2);
//Servo servo;

void setup() {
  Serial.begin(9600);
  Serial3.begin(4800);
  myTransfer.begin(Serial3);
  pinMode(STBY, OUTPUT);
}

void loop() {
  if (myTransfer.available()) {
    myTransfer.rxObj(payload);
    mapX1 = payload.pos_x1;
    mapY1 = payload.pos_y1;
    bool btnOld1 = btn1;
    btn1 = payload.sw1;

    mapX2 = payload.pos_x2;
    mapY2 = payload.pos_y2;
    bool btnOld2 = btn2;
    btn2 = payload.sw2;

    if (btn1 != btnOld1) {
      if (btn1) {
        brake = !brake;
      }
    }

    if (brake) {
      robot.stop();
      digitalWrite(STBY, LOW);
    } else {
      digitalWrite(STBY, HIGH);
      if (mapY1 >= 0) {
        robot.forward();
      } else {
        robot.backward();
        mapY1 *= -1;
      }
      if (mapX1 >= 0) {
        robot.setSpeedB(mapY1);
        robot.setSpeedA(mapY1 - mapX1);
      } else {
        mapX1 *= -1;
        robot.setSpeedB(mapY1 - mapX1);
        robot.setSpeedA(mapY1);
      }
    }
  }
}
