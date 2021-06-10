#include <Rotary.h>
#include <LiquidCrystal_I2C.h>
#include <L298NX2.h>
#include <Ultrasonic.h>

// Motor driver pins
#define MOTOR_A1 2
#define MOTOR_A2 3
#define MOTOR_B1 4
#define MOTOR_B2 5
#define MOTOR_A_EN 6
#define MOTOR_B_EN 7
#define STBY 9

// Ultrasound sensor pins
#define L_TRIG 33
#define L_ECHO 32
#define R_TRIG 35
#define R_ECHO 34
#define F_TRIG 37
#define F_ECHO 36
#define B_TRIG 39
#define B_ECHO 38

// Rotary encoder pins
#define ENC_A 12
#define ENC_B 11
#define ENC_SW 10

#define impactSensor 18

// Control variables
volatile bool impact = 0;
volatile unsigned long impactTime;
bool leftTrack = 0;

// Menu variables
int pos = 0;
int lastPos = pos;
int progress = 1;
int mode = 0;

Rotary rotary = Rotary(ENC_A, ENC_B);
LiquidCrystal_I2C lcd(0x27, 16, 2);

L298NX2 robot (MOTOR_B_EN, MOTOR_B1, MOTOR_B2, MOTOR_A_EN, MOTOR_A1, MOTOR_A2);
Ultrasonic sonarL (L_TRIG, L_ECHO);
Ultrasonic sonarR (R_TRIG, R_ECHO);
Ultrasonic sonarF (F_TRIG, F_ECHO);
Ultrasonic sonarB (B_TRIG, B_ECHO);

void setup() {
    lcd.init();
    lcd.backlight();
    lcd.noAutoscroll();

    pinMode(ENC_SW, INPUT_PULLUP);
    pinMode(STBY, OUTPUT);
    
    while (digitalRead(ENC_SW)) {
        updateLCD();
        delay(200);
        Serial.println("main loop");
    }

    switch (mode) {
        case 0:
            break;
        case 1:
            avoidObstacles();
    }
}

void loop() {}

/********************************* OBSTACLE AVOIDANCE START **************************************/

void avoidObstacles() {
    // Setup
    digitalWrite(STBY, HIGH);

    // Configure impact sensor pins & attach interrupt
    pinMode(impactSensor, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    attachInterrupt(digitalPinToInterrupt(impactSensor), onImpact, FALLING);

    robot.setSpeed(255);
    robot.forward();

    // Loop
    for(;;) {
        checkImpact();
        while (!detectObstacle()) {
            robot.setSpeed(255);
            robot.forward();
        }
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

bool detectObstacle() {
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

/********************************* OBSTACLE AVOIDANCE END ****************************************/

/*************************************** MENU START **********************************************/

int getEncoder() {
  unsigned char dir = rotary.process();
  if (dir == DIR_CW) {
    pos++;
  } else if (dir == DIR_CCW) {
    pos--;
  }
  if (pos < 0) {
    //pos = 0;
  }
  return pos;
}

void clearLine(int line) {
    for (int i = 0; i < 16; i++) {
        lcd.setCursor(i, line);
        lcd.print(' ');
    }
}

void finish() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("All-set ;)"));
    lcd.setCursor(0, 1);
    lcd.print("Enjoy!");
    delay(3000);
}

void welcome() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Welcome to Alset"));
    lcd.setCursor(0, 1);
    lcd.print(F("Click to proceed"));
    while (digitalRead(ENC_SW)) {};
    progress = 2;
}

void askSettings() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Change settings?"));
    lcd.setCursor(0, 1);
    lcd.print(F("No"));
    delay(300);
    
    getEncoder();
    lastPos = pos;
    while (digitalRead(ENC_SW)) {
        unsigned long unDelay = millis();
        while (millis() - unDelay < 500) {
            getEncoder(); 
        }
        if (pos != lastPos) {
            lastPos = pos;
            clearLine(1);
            lcd.setCursor(0, 1);
            if ((pos % 2) == 0) { // Allows constant rotation
                lcd.print(F("No"));
            }
            else {
                lcd.print(F("Yes"));
            }
        }
    }
        
    if ((pos % 2) == 0) {
        progress = 0;
    }
    else {
      progress = 3;
    }
}

void baseMode() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Select base mode:"));
    lcd.setCursor(0, 1);
    lcd.print(F("Line following"));
    delay(100);
    
    pos = getEncoder();
    lastPos = pos;
    while (digitalRead(ENC_SW)) {
        unsigned long unDelay = millis();
        while (millis() - unDelay < 500) {
            getEncoder(); 
        }
        if (pos != lastPos) {
            lastPos = pos;
            clearLine(1);
            lcd.setCursor(0, 1);
            printBaseModes();
        }
    }
    mode = pos;
    progress = 0;
}

void printBaseModes() {
    switch (pos) {
        case 0:
            lcd.print(F("Line following"));
            break;
        case 1:
            lcd.print(F("Avoid obstacles"));
            break;
        case 2:
            lcd.print(F("WiFi joystick"));
            break;
        case 3:
            lcd.print(F("Phone bluetooth"));
            break;
        case 4:
            lcd.print(F("Machine vision"));
            break;
        default:
            if (pos > 4) {
                pos = 0;
            }
            else if (pos < 0) {
                pos = 4;
            }
    }
}

void updateLCD() {
    switch (progress) {
        case 0: // Setup finished screen
            finish();
            break;
        case 1: // Welcome screen
            welcome();
            break;
        case 2: // Setup choice screen
            askSettings();
            break;
        case 3: // Base mode choice screen
            baseMode();
            break;
    }
}

/**************************************** MENU END ***********************************************/
