#include <Ultrasonic.h>
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>
#include "I2CTransfer.h"

#define L_ECHO 8
#define L_TRIG 9
#define R_ECHO 6
#define R_TRIG 7
#define TIMEOUT 2250 // 40 cm

Ultrasonic sonarL(L_TRIG, L_ECHO, TIMEOUT);
Ultrasonic sonarR(R_TRIG, R_ECHO, TIMEOUT);

struct __attribute__((__packed__)) MixedStruct {
    uint8_t distanceL;
    uint8_t distanceR;
    double lat;
    double lng;
    double speed;
    char dir;
} mixedData;

#define GPS_BAUD 9600

TinyGPSPlus gps;
TinyGPSCustom steerDirection(gps, "GPRMB", 3);

LiquidCrystal_I2C lcd(0x27,20,4);

I2CTransfer myTransfer;

unsigned long lastUpdate = millis();
short count = 0;

void recvData() {
    uint16_t recSize = 0;
    //recSize = myTransfer.rxObj(testStruct, recSize);
}

const functionPtr callbackArr[] = { recvData };

void setup() {
    Serial1.begin(GPS_BAUD);
    Wire.begin();

    configST myConfig;
    myConfig.debug        = true;
    myConfig.callbacks    = callbackArr;
    myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
    
    myTransfer.begin(Wire, myConfig);

    lcd.init();
    lcd.backlight();
}

static void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (Serial1.available())
            gps.encode(Serial1.read());
    } while (millis() - start < ms);
}

void readSonars() {
    mixedData.distanceL = sonarL.read();
    mixedData.distanceR = sonarR.read();
}

void readGPS() {
    if (gps.location.isUpdated()) {
        mixedData.lat = gps.location.lat();
        mixedData.lng = gps.location.lng();
        mixedData.dir = steerDirection.value();
    }
    if (gps.speed.isUpdated()) {
        mixedData.speed = gps.speed.mps();
    }
}

void sendData() {
    uint16_t sendSize = 0;
    sendSize = myTransfer.txObj(mixedData, sendSize);
    myTransfer.sendData(sendSize);
}

void updateScreen() {
    if (millis() - lastUpdate >= 1200) {
        lcd.clear();
        smartDelay(100);
        if (count == 0) {
            lcd.setCursor(0, 0);
            lcd.print(F("Ultrasonic"));
            lcd.setCursor(0, 1);
            lcd.print(F("Left: "));
            lcd.print(mixedData.distanceL);
            lcd.setCursor(0, 2);
            lcd.print(F("Right: "));
            lcd.print(mixedData.distanceR);
            count++;
        }
        else if (count == 1) {
            lcd.setCursor(0, 0);
            lcd.print(F("GPS"));
            lcd.setCursor(0, 1);
            lcd.print(F("Latitude: "));
            lcd.setCursor(10, 1);
            lcd.print(mixedData.lat);
            lcd.setCursor(0, 2);
            lcd.print(F("Longitude: "));
            lcd.setCursor(10, 2);
            lcd.print(mixedData.lng);
            lcd.setCursor(0, 3);
            if (mixedData.lat == 0 && mixedData.lng == 0)
                lcd.print(F("Signal unavailable"));
            if (mixedData.dir == 'L') {
                lcd.print(F("Steer left"));
            }
            else if (mixedData.dir == 'R') {
                lcd.print(F("Steer right"));
            }
            count++;
        }
        else {
            count = 0;
            updateScreen();
        }
        lastUpdate = millis();
    }
}

void loop() {
    smartDelay(50);
    readSonars();
    smartDelay(50);
    readGPS();
    smartDelay(50);
    sendData();
    updateScreen();
}
