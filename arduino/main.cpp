// github repo at https://github.com/robert-saramet/Line-Follower
// using L298N library found at https://github.com/AndreaLombardo/L298N
// using 5 sensor cytron maker line (digital inputs)

/*
 ----------------   ----------------------    
 | Mega | L298N |   | Mega | Line Sensor |
 |------|-------|   |------|-------------|
 |  2   |  IN1  |   |  22  |      D1     |
 |  3   |  IN2  |   |  23  |      D2     |
 |  4   |  IN3  |   |  24  |      D3     |
 |  5   |  IN4  |   |  25  |      D4     |
 |  6   |  ENA  |   |  26  |      D5     |
 |  7   |  ENB  |   |---------------------
 ----------------   
*/

#include <Arduino.h>
#include <L298NX2.h>

#define motorA1 2
#define motorA2 3
#define motorB1 4
#define motorB2 5
#define motorA_EN 6
#define motorB_EN 7

#define rxPin 8

short ls[6][2]; //ls[1-5][0] for line sensor pin numbers, ls[1-5][1] for sensor values
int firstPin = 22; //first line sensor pin number
float speedMultiplier = 1; //number to divide speed by

L298NX2 robot(motorA_EN, motorA1, motorA2, motorB_EN, motorB1, motorB2); //initialize motor driver

void steer(int speedA, int SpeedB){
    robot.setSpeedA(speedA / speedMultiplier);
    robot.setSpeedB(SpeedB /speedMultiplier);
}

//get value of each line sensor and store it in array
void readLine(){
    for(int i = 1; i <= 5; i++){
        ls[i][1] = digitalRead(ls[i][0]);
    }
}

void followLine(){
    readLine();
    if (ls[1][1]==0 && ls[2][1]==0 && ls[3][1]==1 && ls[4][1]==0 && ls[5][1]==0){ 
        robot.setSpeed(255 / speedMultiplier);
        robot.forward();
    }
    else if (ls[1][1]==0 && ls[2][1]==1 && ls[3][1]==1 && ls[4][1]==1 && ls[5][1]==0){ 
        robot.setSpeed(255 / speedMultiplier);
        robot.forward();
    }
    else if (ls[1][1]==0 && ls[2][1]==1 && ls[3][1]==1 && ls[4][1]==0 && ls[5][1]==0){ 
        steer(128, 255);
        robot.forwardA();
        robot.forwardB();
    }
    else if (ls[1][1]==0 && ls[2][1]==1 && ls[3][1]==0 && ls[4][1]==0 && ls[5][1]==0){ 
        steer(64, 255);
        robot.forwardA();
        robot.forwardB();
    }
    else if (ls[1][1]==1 && ls[2][1]==1 && ls[3][1]==0 && ls[4][1]==0 && ls[5][1]==0){ 
        steer(128, 255);
        robot.backwardA();
        robot.forwardB();
    }
    else if (ls[1][1]==1 && ls[2][1]==0 && ls[3][1]==0 && ls[4][1]==0 && ls[5][1]==0){ 
        steer(255, 255);
        robot.backwardA();
        robot.forwardB();
    }
    else if (ls[1][1]==0 && ls[2][1]==0 && ls[3][1]==1 && ls[4][1]==1 && ls[5][1]==0){ 
        robot.forwardA();
        robot.forwardB();
    }
    else if (ls[1][1]==0 && ls[2][1]==0 && ls[3][1]==0 && ls[4][1]==1 && ls[5][1]==0){ 
        steer(255, 64);
        robot.forwardA();
        robot.forwardB();
    }
    else if (ls[1][1]==0 && ls[2][1]==0 && ls[3][1]==0 && ls[4][1]==0 && ls[5][1]==1){ 
        steer(255, 128);
        robot.forwardA();
        robot.backwardB();
    }
    else if (ls[1][1]==0 && ls[2][1]==0 && ls[3][1]==0 && ls[4][1]==0 && ls[5][1]==1){ 
        steer(255, 255);
        robot.forwardA();
        robot.backwardB();
    }
    else if (ls[1][1]==0 && ls[2][1]==1 && ls[3][1]==1 && ls[4][1]==1 && ls[5][1]==0){ 
        robot.setSpeed(255 / speedMultiplier);
        robot.forward();
    }
    else if (ls[1][1]==1 && ls[2][1]==1 && ls[3][1]==1 && ls[4][1]==0 && ls[5][1]==0){ 
        steer(128, 255);
        robot.backwardA();
        robot.forwardB();
    }
    else if (ls[1][1]==0 && ls[2][1]==0 && ls[3][1]==1 && ls[4][1]==1 && ls[5][1]==1){ 
        steer(255, 128);
        robot.forwardA();
        robot.backwardB();
    }
    else{
      //followLine();  if this this is uncommented and robot is off track it will stop receiving commands
    }
}

void setup(){
    Serial.begin(9600);
    robot.setSpeed(255);
    robot.stop();

    //initialize line sensor pins as inputs
    for(int i = 1; i <= 5; i++){
        ls[i][0] = firstPin - 1 + i;
        pinMode(ls[i][0], INPUT);
    }

    pinMode(rxPin, INPUT_PULLUP);
}

void loop() {
    bool halt = !digitalRead(rxPin);
    if (!halt) {
        followLine();
        Serial.println("Following line");
    }
    else {
        robot.stop();
    }
}