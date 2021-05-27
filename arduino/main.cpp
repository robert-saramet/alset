#include <Arduino.h>
#include <L298NX2.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <SerialTransfer.h>
#include <Servo.h>
#include <TinyIRReceiver.h>
#include <LiquidCrystal_I2C.h>


// Set robot features
#define follow_line 0
#define avoid_obstacles 0
#define return_line 0
#define impact_sensor 0
#define comms 0
#define imu 0
#define leds 0
#define autoOff 0
#define serial_log 1
#define ir_remote 1
#define display 1


// Motor driver pins
#define motorA1 2
#define motorA2 3
#define motorB1 4
#define motorB2 5
#define motorA_EN 6
#define motorB_EN 7

// Mixed module pins
#define impactSensor 18
#define MAKERLINE_AN A0
#define laser 11
#define servoPin 9


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


// Control variables
bool leftTrack = 0;
volatile bool impact = 0;
volatile unsigned long impactTime;


// IR Remote setup
#define IR_INPUT_PIN 19
#include "TinyIRReceiver.cpp.h"
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
long debounce = millis();

// IR Remote keys
#define OK 0x1C
#define UP 0x18
#define DOWN 0x52
#define LEFT 0x8
#define RIGHT 0x5A


L298NX2 robot(motorA_EN, motorA1, motorA2, motorB_EN, motorB1, motorB2);
Ultrasonic sonarL(33, 32);
Ultrasonic sonarR(35, 34);
Ultrasonic sonarM(37, 36);
Servo servo;
SerialTransfer myTransfer;
LiquidCrystal_I2C lcd(0x27, 20, 4);  

// For IMU
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#include "NXP_FXOS_FXAS.h"
Adafruit_Madgwick filter;
Adafruit_Sensor_Calibration_EEPROM cal;
#define FILTER_UPDATE_RATE_HZ 100
#define IMU_BUFFER 10
uint32_t timestamp;


// Function forward-declaration
void onImpact();
void checkImpact();
bool detectObstacle();
void followLine();
void returnToLine();
void getJoystick();
int getIR();
void getAngles();


void setup() {
    delay(3000); // Place robot at the center of line
    adcSetPoint = analogRead(MAKERLINE_AN);

    if (serial_log) {
        Serial.begin(9600);
    }

    if (comms) {
        // This connection is used for the joystick
        Serial3.begin(4800);
        myTransfer.begin(Serial3);
    }

    if (impact_sensor) {
        // Configure impact sensor pins & attach interrupt
        pinMode(impactSensor, INPUT);
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, LOW);
        attachInterrupt(digitalPinToInterrupt(impactSensor), onImpact, FALLING);
    }

    if (avoid_obstacles && !(comms || follow_line)) {
      robot.setSpeed(255);
      robot.forward();
    }

    if (ir_remote) {
        initPCIInterruptForTinyReceiver();
    }

    if (imu) {
    // Prepare IMU
      cal.begin();
      cal.loadCalibration();
      init_sensors();

      setup_sensors();
      filter.begin(FILTER_UPDATE_RATE_HZ);
      timestamp = millis();
      Wire.setClock(400000); // 400 KHz
    }

    if (display) {
        lcd.init();
        lcd.backlight();
        lcd.setCursor(0, 0);
        lcd.print("Welcome to Alset");
        lcd.setCursor(0, 1);
        lcd.print("Click to proceed");

        // wait for button press
        lcd.clear();
        // while not button press
            //display lines for current encoder position
        lcd.clear();
        lcd.noBacklight();
    }
}


void loop() {
    checkImpact();
    if (!(comms || ir_remote)) { // When using joystick, switch to manual mode 
        detectObstacle();
        returnToLine();
        followLine();
    }
    else {
        getJoystick();
        getIR();
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
                leftTrack = 1;
            }
        interrupts();
        }
    }
}


bool detectObstacle() {
    if (avoid_obstacles) {
        if(sonarM.read() < 20){
            robot.backward();
            delay(80);
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
    }
    return 1;
}


void returnToLine() {
    if (return_line){
        if (leftTrack) {
            // Check if still on line
            while (analogRead(MAKERLINE_AN) < 51) {
                //Search for line
                robot.setSpeedA(255);   //---------------//
                robot.setSpeedB(160);   // TODO          //
                robot.forwardFor(40);   // USE GYRO HERE //
                robot.reset();          //---------------//
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
                        motorLeft = MAX_SPEED - powerDifference;
                        motorRight = MAX_SPEED;
                    }
                }

                robot.setSpeedA(motorLeft);
                robot.setSpeedB(motorRight);
            }
        }
    }
}


void getJoystick() {
    if (comms) {
        if (myTransfer.available()) {
            // Load packet from ESP
            myTransfer.rxObj(payload);
            // Values for motors
            mapX1 = payload.pos_x1;
            mapY1 = payload.pos_y1;
            bool btnOld1 = btn1;
            btn1 = payload.sw1;

            // Values for servo
            mapX2 = payload.pos_x2;
            mapY2 = payload.pos_y2;
            bool btnOld2 = btn2;
            btn2 = payload.sw2;

            // Check brake button toggle
            if (btn1 != btnOld1) {
                if (btn1) {
                    brake = !brake;
                }
            }

            if (brake) {  // Press joystick button to stop robot
                robot.stop();
            } else {
                // Set robot speed & direction
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

            // Change servo position
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

            // Check laser button toggle
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
}

int getIR(uint16_t irAdress, uint8_t irCommand, bool isRepeat) {
    if (ir_remote) {
        if (irCommand == OK) {
            if (!isRepeat) {
                if (debounce - millis() > 100) {
                    brake != brake;
                    debounce = millis();
                } 
            }
        }
        else {
            if (!brake) {
                robot.setSpeed(255);
                switch (irCommand) {
                    case UP:
                        robot.forward();
                        break;
                    case DOWN:
                        robot.backward();
                        break;
                    case LEFT:
                        robot.backwardA();
                        robot.forwardB();
                        break;
                    case RIGHT:
                        robot.forwardA();
                        robot.backwardB();
                        break;
                }
            }
        }
    }
    return irCommand;
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

        // Read calibration data from EEPROM
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

        // Only returns the calculated output once in a while
        if (counter++ <= IMU_BUFFER) {
            return;
        }
        counter = 0;

        roll = filter.getRoll();
        pitch = filter.getPitch();
        heading = filter.getYaw();
    }
}
