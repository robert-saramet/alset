#include <Arduino.h>
#include <L298NX2.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <SerialTransfer.h>
#include <Servo.h>


#define motorA1 2
#define motorA2 3
#define motorB1 4
#define motorB2 5
#define motorA_EN 6
#define motorB_EN 7

#define impactSensor 18
#define MAKERLINE_AN A0

#define laser 11
#define servoPin 9

// Set robot features
bool follow_line = 1;
bool avoid_obstacles = 0;
bool return_line = 0;
bool impact_sensor = 0;
bool comms = 0;
bool imu = 0;


// Joystick variables
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


// PD variables
byte MAX_SPEED = 255;
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


// IMU variables
float roll, pitch, heading;


bool leftTrack = 0;
volatile bool impact = 0;
volatile unsigned long impactTime;


L298NX2 robot(motorA_EN, motorA1, motorA2, motorB_EN, motorB1, motorB2);
Ultrasonic sonarL(33, 32);
Ultrasonic sonarR(35, 34);
Ultrasonic sonarM(37, 36);
Servo servo;
SerialTransfer myTransfer;

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#include "NXP_FXOS_FXAS.h"
Adafruit_Madgwick filter;
Adafruit_Sensor_Calibration_EEPROM cal;
#define FILTER_UPDATE_RATE_HZ 100
#define IMU_BUFFER 10
uint32_t timestamp;


void onImpact() {
    digitalWrite(LED_BUILTIN, HIGH);
    robot.setSpeed(255);
    robot.backward();
    impact = 1;
    impactTime = millis();
}


void checkImpact() {
    if (impact_sensor) {
        noInterrupts();
        if (impact) {
            if (millis() - impactTime > 300) {
                impact = 0;
                robot.stop();
                leftTrack = 1;
            }
        interrupts();
        }
    }
}


void checkObstacle() {
    if (avoid_obstacles) {
        if(sonarM.read() < 15){
            robot.backwardFor(100);
            robot.reset();
        }
        if (sonarL.read() < 25 || sonarR.read() < 25) {
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
            else {
                robot.backwardFor(300);
                robot.reset();
            }
        }
    }
}


void returnToLine() {
    if (return_line){
        if (leftTrack) {
            // Check if still on line
            while (analogRead(MAKERLINE_AN) < 51) {
                //Search for line
                robot.setSpeedA(255);
                robot.setSpeedB(160);
                robot.forwardFor(40);
                robot.reset();
            }
        leftTrack = 0;
        }
    }
}


void followLine() {
    if (follow_line) {
        currentMillis = millis();
        if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            adcMakerLine = analogRead(MAKERLINE_AN);

            if (adcMakerLine < 103) {
                // Outside line
                robot.setSpeed(0);
            }
            else if (adcMakerLine > 972) { 
                // Detects cross line
                robot.setSpeedA(MAX_SPEED - 25);
                robot.setSpeedB(MAX_SPEED - 25);
            }
            else {
                proportional = adcMakerLine - adcSetPoint;
                derivative = proportional - lastProportional;
                lastProportional = proportional;

                powerDifference = (proportional * 1.5) + (derivative * 5);

                // Go forward at full speed
                if (powerDifference > MAX_SPEED) {
                    powerDifference = MAX_SPEED;
                }
                // Go backward at full speed
                if (powerDifference < -MAX_SPEED) {
                    powerDifference = -MAX_SPEED;
                }

                // Turn right
                if (powerDifference < 0) {
                    motorLeft = MAX_SPEED + powerDifference;
                    motorRight = MAX_SPEED;
                }
                // Turn left
                else {
                    motorLeft = MAX_SPEED;
                    motorRight = MAX_SPEED - powerDifference;
                }

                robot.setSpeedA(motorLeft);
                robot.setSpeedB(motorRight);
                robot.forward();
            }
        }
    }
}


void getJoystick() {
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

    //Serial.print("mapX: ");
    //Serial.println(mapX);
    Serial.print("mapY2: ");
    Serial.println(mapY2);
    Serial.print("btn2: ");
    Serial.println(btn2);
    

    if (btn1 != btnOld1) {
      if (btn1) {
        brake = !brake;
      }
    }

    if (brake) {
      robot.stop();
    } else {
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

    if (mapX2 > 100 || mapX2 < -100) {
      if (mapX2 < 0) {
        if (pos >= 2) {
          pos -= 2;
        }
      } else {
        if (pos <= 168) {
          pos += 2;
        }
      }
      servo.write(pos);
    }

    if (btn2 != btnOld2) {
      if (btn2) {
        pwr = !pwr;
      }
      if (pwr) {
        digitalWrite(laser, HIGH);
      } else {
        digitalWrite(laser, LOW);
      }
    } 
  }
}


void getAngles() {
    if (imu) {
        float gx, gy, gz;
        static uint8_t counter = 0;

        if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
            return;
        }

        timestamp = millis();

        // Read the motion sensors
        sensors_event_t accel, gyro, mag;
        accelerometer->getEvent(&accel);
        gyroscope->getEvent(&gyro);
        magnetometer->getEvent(&mag);

        cal.calibrate(mag);
        cal.calibrate(accel);
        cal.calibrate(gyro);

        // Gyroscope needs to be converted from Rad/s to Degree/s
        // the rest are not unit-important
        gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
        gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
        gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

        // Update the SensorFusion filter
        filter.update(gx, gy, gz, 
                        accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                        mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

        // only print the calculated output once in a while
        if (counter++ <= IMU_BUFFER) {
            return;
        }
        counter = 0;

        roll = filter.getRoll();
        pitch = filter.getPitch();
        heading = filter.getYaw();
    }
}


void setup() {
    delay(2000); // Place robot at the center of line
    adcSetPoint = analogRead(MAKERLINE_AN);

    if (comms) {
        Serial3.begin(4800);
        myTransfer.begin(Serial3);
    }

    if (impact_sensor) {
        pinMode(impactSensor, INPUT);
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, LOW);
        attachInterrupt(digitalPinToInterrupt(impactSensor), onImpact, FALLING);
    }

    if (imu) {
      cal.begin();
      cal.loadCalibration();
      init_sensors();

      accelerometer->printSensorDetails();
      gyroscope->printSensorDetails();
      magnetometer->printSensorDetails();

      setup_sensors();
      filter.begin(FILTER_UPDATE_RATE_HZ);
      timestamp = millis();
      Wire.setClock(400000); // 400 KHz
    }
}


void loop() {
    checkImpact();
    if (!comms) {
        checkObstacle();
        returnToLine();
        followLine();
    }
    else {
        getJoystick();
    }
}
