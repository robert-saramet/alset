#include <L298NX2.h>

#define motorA1 2
#define motorA2 3
#define motorB1 5
#define motorB2 4
#define motorA_EN 6
#define motorB_EN 7
#define STBY 9

#define MAKERLINE_AN  A0
#define MAX_SPEED 255

// PD control variables
int adcMakerLine = 0;
int adcSetPoint = 0;
int proportional = 0;
int lastProportional = 0;
int derivative = 0;
int powerDifference = 0;
int motorLeft = 0;
int motorRight = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const int interval = 10;


L298NX2 robot(motorB_EN, motorB1, motorB2, motorA_EN, motorA1, motorA2); //initialize motor driver

void setup () {
  delay(2000);
  // Place robot at the center of line
  adcSetPoint = analogRead(MAKERLINE_AN);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void loop()
{
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    adcMakerLine = analogRead(MAKERLINE_AN);

    if (adcMakerLine < 51) { // Out of line
      //robot.setSpeed(0);
    }
    else if (adcMakerLine > 972) { // Detects cross line
      robot.setSpeedA(MAX_SPEED - 25);
      robot.setSpeedB(MAX_SPEED - 25);
    }
    else {
      proportional = adcMakerLine - adcSetPoint;
      derivative = proportional - lastProportional;
      lastProportional = proportional;

      powerDifference = (proportional * 1) + (derivative * 4);

      if (powerDifference > MAX_SPEED) {
        robot.forwardA();
        robot.backwardB();
        motorLeft = MAX_SPEED;
        if (powerDifference < 2 * MAX_SPEED) {
          motorRight = powerDifference - MAX_SPEED;
        }
        else {
          motorRight = MAX_SPEED;
        }
      }

      else if (powerDifference < -MAX_SPEED) {
        robot.backwardA();
        robot.forwardB();
        motorRight = MAX_SPEED;
        if (powerDifference > 2 * -MAX_SPEED) {
          motorLeft = -MAX_SPEED - powerDifference;
        }
        else {
          motorLeft = MAX_SPEED;
        }
      }

      else if (powerDifference < MAX_SPEED  && powerDifference > -MAX_SPEED) {
        robot.forward();
        if (powerDifference > 0) {
          motorLeft = MAX_SPEED;
          motorRight = MAX_SPEED - powerDifference;
        }
        else {
          motorLeft = powerDifference - MAX_SPEED;
          motorRight = MAX_SPEED;
        }
      }

      robot.setSpeedA(motorLeft);
      robot.setSpeedB(motorRight);
    }
  }
}
