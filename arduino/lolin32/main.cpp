#include <esp_now.h>
#include <WiFi.h>

#undef DEBUG

#define joyR_Btn 13
#define joyR_Y 39
#define joyR_X 36
#define joy_Vcc 27

struct Joystick {
    int x, y;
    bool btn;
} joyR;

struct JoyCal {
    int minX, minY, maxX, maxY;
} calR = {4095, 4095, 0, 0};

struct RemoteData {
    int relSpeed, pos;
    bool turbo, autoPilot;
} remDat = {0, 90, 0, 0};

uint8_t rxAddr[] = {0x24, 0x6F, 0x28, 0x8F, 0x53, 0x58};

void readJoystick() {
    int valX = analogRead(joyR_X);
    int valY = analogRead(joyR_Y);
    joyR.x = map(valX, calR.minX, calR.maxX, 0, 180);
    joyR.y = map(valY, calR.minY, calR.maxY, 100, -100);
    if ((joyR.x > 80) && (joyR.x < 100))
        joyR.x = 90;
    if ((joyR.y > -10) && (joyR.y < 11))
        joyR.y = 0;
    joyR.btn = !digitalRead(joyR_Btn);
}

void readJoystickRaw() {
    joyR.x = analogRead(joyR_X);
    joyR.y = analogRead(joyR_Y);
    joyR.btn = !digitalRead(joyR_Btn);
}

void setRange() {
    readJoystickRaw();
    while (!joyR.btn) {
        if (joyR.x < calR.minX)
            calR.minX = joyR.x;
        else if (joyR.x > calR.maxX)
            calR.maxX = joyR.x;
        if (joyR.y < calR.minY)
            calR.minY = joyR.y;
        else if (joyR.y > calR.maxY)
            calR.maxY = joyR.y;
        readJoystickRaw();
    }
}

void initJoystick() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(joyR_Btn, INPUT_PULLUP);
    pinMode (joyR_X, INPUT);
    pinMode(joyR_Y, INPUT);
    dacWrite(26, 190);
    delay(300);
    setRange();
}

void printDebug() {
    #ifdef DEBUG
        Serial.print(F("X: "));
        Serial.print(joyR.x);
        Serial.print(F("\t\tY: "));
        Serial.print(joyR.y);
        Serial.print(F("\t\tBtn: "));
        Serial.println(joyR.btn);
        Serial.println();
    #endif
}

void OnDataSent (const uint8_t *mac_addr, esp_now_send_status_t status) {
    #ifdef DEBUG
        Serial.print(F("\r\nLast Packet Send Status:\t"));
        Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Successful" : "Delivery Failed");
    #endif
}

void initComms() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        #ifdef DEBUG
            Serial.println(F("Error initializing ESP-NOW"));
        #endif
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, rxAddr, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        #ifdef DEBUG
            Serial.println("Failed to add peer");
        #endif
        return;
    }
}

void sendData() {
    remDat.relSpeed = joyR.y;
    remDat.pos = joyR.x;
    remDat.turbo = joyR.btn;
    esp_err_t result = esp_now_send(rxAddr, (uint8_t *) &remDat, sizeof(remDat));
    #ifdef DEBUG
        if (result == ESP_OK) {
            Serial.println(F("Sent succesfully"));
        } else {
            Serial.println(F("Error sending data"));
        }
    #endif
}

void setup() {
    Serial.begin(115200);
    initJoystick();
    initComms();
}

void loop() {
    readJoystick();
    sendData();
    printDebug();
    delay(100);
}
