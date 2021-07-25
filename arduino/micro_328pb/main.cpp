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
LiquidCrystal_I2C lcd(0x27,20,4);
I2CTransfer myTransfer;

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
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Ultrasonic"));
    lcd.setCursor(0, 1);
    lcd.print(F("Left: "));
    lcd.print(mixedData.distanceL);
    lcd.setCursor(0, 2);
    lcd.print(F("Right: "));
    lcd.print(mixedData.distanceR);
}

void loop() {
    smartDelay(50);
    readSonars();
    smartDelay(50);
    readGPS();
    smartDelay(50);
    sendData();
    smartDelay(50);
    updateScreen();
}
