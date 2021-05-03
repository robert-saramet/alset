#include <Arduino.h>
#include <L298NX2.h>
#include <Ultrasonic.h>

#define motorA1 2
#define motorA2 3
#define motorB1 4
#define motorB2 5
#define motorA_EN 6
#define motorB_EN 7

L298NX2 robot(motorA_EN, motorA1, motorA2, motorB_EN, motorB1, motorB2);
Ultrasonic sonarL(33, 32);
Ultrasonic sonarR(35, 34);

void setup(){
  robot.setSpeed(255);
}

void loop(){
  int distL = sonarL.read();
  int distR = sonarR.read();

  if(distL > 20 && distR > 20){
    robot.forward();
  }

  else{
    if(distL < distR){
        robot.forwardA();
        robot.backwardB();
    }
    else if(distL > distR){
        robot.backwardA();
        robot.forwardB();
    }
    else{
        robot.backwardFor(400);
        robot.reset();
    }
  }
}
