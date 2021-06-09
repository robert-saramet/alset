#include <Arduino.h>
#include <L298NX2.h>
#include <Ultrasonic.h>

#define motorA1 2
#define motorA2 3
#define motorB1 5
#define motorB2 4
#define motorA_EN 6
#define motorB_EN 7
#define STBY 9

#define impactSensor 18

// Set robot features
#define avoid_obstacles 1
#define impact_sensor 0

// Control variables
volatile bool impact = 0;
volatile unsigned long impactTime;
bool leftTrack = 0;

L298NX2 robot(motorB_EN, motorB1, motorB2, motorA_EN, motorA1, motorA2);
Ultrasonic sonarL(33, 32);
Ultrasonic sonarR(35, 34);
Ultrasonic sonarF(37, 36);
Ultrasonic sonarB(39, 38);

// Function forward-declaration
void onImpact();
void checkImpact();
bool detectObstacle();

void setup() {
    delay(3000); // Place robot at the center of line

    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    if (impact_sensor) {
        // Configure impact sensor pins & attach interrupt
        pinMode(impactSensor, INPUT);
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, LOW);
        attachInterrupt(digitalPinToInterrupt(impactSensor), onImpact, FALLING);
    }

    if (avoid_obstacles) {
      robot.setSpeed(255);
      robot.forward();
    }
}

void loop() {
    checkImpact();
    while (!detectObstacle()) {
        robot.setSpeed(255);
        robot.forward();
    }
}

// Handle interrupt
void onImpact() {
    digitalWrite(LED_BUILTIN, HIGH);
    robot.setSpeed(255);
    robot.backward();
    impact = 1;
    impactTime = millis();
}

// Check impact state
void checkImpact() {
    if (impact_sensor) {
        noInterrupts(); // Avoid interference
        if (impact) {
            if (millis() - impactTime > 300) {
                impact = 0;
                robot.stop();
                digitalWrite(STBY, LOW);
                leftTrack = 1;
            }
        interrupts();
        }
    }
}

bool detectObstacle() {
    if (avoid_obstacles) {
      if (sonarB.read() > 5) {
        if (sonarF.read() < 20){
            if (sonarB.read() > 10) {
                robot.backward();
                delay(80);
            }
            else {
              robot.backwardForA(80);
              robot.forwardForB(80);
              robot.reset();
            }
            return 0;
        }
        if (sonarL.read() < 30 || sonarR.read() < 30) {
            leftTrack = 1;
            robot.setSpeed(255);
            if (sonarL.read() < sonarR.read()) {
                while (sonarL.read() < 35) {
                    robot.forwardA();
                    robot.backwardB();
                }
            }
            else if (sonarL.read() > sonarR.read()) {
                while (sonarR.read() < 35) {
                    robot.forwardB();
                    robot.backwardA();
                }
            }
            else { // If both sensors return same value, to avoid getting stuck
                robot.backwardFor(300);
                robot.reset();
            }
            return 0;
        }
        return 1;
      }
    }
}
