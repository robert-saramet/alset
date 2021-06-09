#include <Rotary.h>
#include <LiquidCrystal_I2C.h>

#define ENC_A 12
#define ENC_B 11
#define ENC_SW 10

Rotary rotary = Rotary(ENC_A, ENC_B);
LiquidCrystal_I2C lcd(0x27, 16, 2);

int pos = 0;
int lastPos = pos;
int progress = 1;
int mode = 0;

void setup() {
    lcd.init();
    lcd.backlight();
    lcd.noAutoscroll();

    Serial.begin(9600);
    pinMode(ENC_SW, INPUT_PULLUP);
    
    while (digitalRead(ENC_SW)) {
        updateLCD();
        delay(200);
        Serial.println("main loop");
    }
}

void loop() {
}

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
    Serial.println(F("Entered FINISH"));
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
        Serial.println(F("WHILE statement reached"));
        Serial.print(F("POS: "));
        Serial.println(pos);
        if (pos != lastPos) {
            lastPos = pos;
            clearLine(1);
            lcd.setCursor(0, 1);
            if ((pos % 2) == 0) { // Allows constant rotation
                lcd.print(F("No"));
                Serial.println(F("NO -  don't change"));
            }
            else {
                lcd.print(F("Yes"));
                Serial.println(F("YES - change"));
            }
            Serial.println(F("IF statement reached"));
        }
    }


    Serial.print(F("POS: "));
    Serial.println(pos);
        
    if ((pos % 2) == 0) {
        progress = 0;
        Serial.println(F("Going to FINISH"));
    }
    else {
      progress = 3;
      Serial.println(F("Going to MODES"));
    }
}

void baseMode() {
    Serial.println("Entered MODES");
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
    Serial.println("FINISHED MODES");
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
            Serial.println("Before finish");
            finish();
            Serial.println("After finish");
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
