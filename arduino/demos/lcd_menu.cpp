#include <Rotary.h>
#include <LiquidCrystal_I2C.h>

#define ENC_A 12
#define ENC_B 11
#define ENC_SW 10

Rotary rotary = Rotary(ENC_A, ENC_B);
LiquidCrystal_I2C lcd(0x27, 16, 2);

int position = 0;
int progress = 1;
bool choice = 0;
int mode = 0;

void setup() {
    Serial.begin(57600);
    
    lcd.init();
    lcd.backlight();

    pinMode(ENC_SW, INPUT);
    
    while (!digitalRead(ENC_SW)) {
        updateLCD();
    }
        getEncoder();
    }
  
}

void loop() {
}

int getEncoder() {
  unsigned char dir = rotary.process();
  if (dir == DIR_CW) {
    position++;
  } else if (dir == DIR_CCW) {
    position--;
  }
  if (position < 0) {
    //position = 0;
  }
  return position;
}

void clearLine(int line) {
    lcd.setCursor(0, line);
    for (int i = 0; i < 16; i++);
        lcd.print(" ");
}

void finish() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("All-set"));
    lcd.setCursor(0, 1);
    lcd.print("Enjoy");
}

void welcome() {
    lcd.setCursor(0, 0);
    lcd.print(F("Welcome to Alset"));
    lcd.setCursor(0, 1);
    lcd.print(F("Click to continue"));
    while (!digitalRead(ENC_SW)) {};
    progress = 2;
}

void askSettings() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Change settings?"))
    lcd.setCursor(0, 1);
    lcd.print(F("No"));

    while (!digitalRead(ENC_SW)) {
        if (position != getEncoder()) {
            clearLine(1);
            lcd.setCursor(0, 1);
            if (position % 2) { // Allows constant rotation
                lcd.print(F("No"));
                choice = 0;
            }
            else {
                lcd.print(F("Yes"));
                choice = 1;
            }
        }
    }

    if (choice) {
        progress = 3;
    }
}

void baseMode() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Select base mode:"));
    lcd.setCursor(0, 1);
    lcd.print(F("Line following"));
    
    position = 0;
    while (!digitalRead(ENC_SW)) {
        if (position != getEncoder()) {
            clearLine(1);
            lcd.setCursor(0, 1);
            printBaseModes();
            }
        }
    }
    mode = position;
}

void printBaseModes() {
    switch (position) {
        case 0:
            lcd.print(F("Line following"));
            break;
        case 1:
            lcd.print(F("Obstacle detection"));
            break;
        case 2:
            lcd.print(F("WiFi joystick"));
            break;
        case 3:
            lcd.print(F("Phone bluetooth"));
            break:
        case 4:
            lcd.print(F("Machine vision"));
            break;
        default:
            if (position > 4) {
                position = 0;
            }
            else if (position < 0) {
                position = 4;
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
