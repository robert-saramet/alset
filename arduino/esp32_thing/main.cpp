//  Libraries
#include "EEPROM.h"
#include "SerialTransfer.h"
#include "PS4Controller.h"
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

// Import Blynk
#define BLYNK_USE_DIRECT_CONNECT
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

// Blynk API key
char auth[] = "r70VT9kv6x1BQJ_iCe7zRaqItWJf_OBS";

// Disable debug data
#undef DEBUG

// Create serial transfer object
SerialTransfer myTransfer;

// Motor structure
struct __attribute__((__packed__)) MotorStruct {
    int8_t speed;
    uint8_t pos;
    bool turbo;
} motors;

// Sonar structure
struct __attribute__((__packed__)) SonarStruct {
    uint8_t distF;
    uint8_t distFL;
    uint8_t distFR;
} sonars;

// GPS & sonar structure
struct __attribute__((__packed__)) MixedStruct {
    uint8_t distanceL;
    uint8_t distanceR;
    double lat;
    double lng;
    double speed;
    char dir;
} mixedData;


// IMU structure
struct __attribute__((__packed__)) GyroStruct {
    float roll;
    float pitch;
    float heading;
} imu;

// Old data structure
struct OldData {
    struct SonarStruct sonars;
    struct MixedStruct mixedData;
    struct MotorStruct motors;
    bool manual;
} old {{60, 60, 60}, {40, 40, 0, 0, 0, 'L'}, {0, 90, 0}, 1};


// PS4 button states
bool turbo = 0;
bool lastCross = 0;
bool lastSquare = 0;

// Robot states
volatile bool updateCon = 0;
bool conMode = 1;
bool brake = 1;
bool manual = 0;
bool fullAuto = 0;
int maxSpeed = 50;

// Sonar values
short distF;
short distFL;
short distFR;
short distL;
short distR;

// Calculated sonar values
long distLFL;
long distRFR;

// Calculated motor values
int mapX = 90;
int mapY = 0;

// IMU values
float gx, gy, gz;
short counter = 0;

// Track time between PS4 updates
long lastSend = millis();

// Create IMU object
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// Include IMU library
#include "NXP_FXOS_FXAS.h"

// Create IMU filter object
Adafruit_NXPSensorFusion filter;

// Load IMU calibration data
#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

// Set IMU processing variables
#define FILTER_UPDATE_RATE_HZ 100
#define POLL_RATE 10

// Track time between IMU updates
uint32_t timestamp;

// Create IMU task
TaskHandle_t GyroTask;

// Setup IMU
void setupIMU() {
        xTaskCreatePinnedToCore(
        GyroTaskCode,    // Function that should be called
        "Update Gyro",   // Name of the task (for debugging)
        1000,            // Stack size (bytes)
        NULL,            // Parameter to pass
        1,               // Task priority
        NULL,             // Task handle
        0                // Core to run task on
  );
  
    if (!cal.begin()) {
        #ifdef DEBUG
            Serial.println("Failed to initialize calibration helper");
        #endif
    } else if (! cal.loadCalibration()) {
        #ifdef DEBUG
            Serial.println("No calibration loaded/found");
        #endif
    }

    if (!init_sensors()) {
        #ifdef DEBUG
            Serial.println("Failed to find sensors");
        #endif
        while (1) delay(10);
    }
  
    #ifdef DEBUG
        accelerometer->printSensorDetails();
        gyroscope->printSensorDetails();
        magnetometer->printSensorDetails();
    #endif

    setup_sensors();
    filter.begin(FILTER_UPDATE_RATE_HZ);
    timestamp = millis();

    Wire.setClock(400000); // 400KHz
}

// IMU task
void GyroTaskCode(void * parameter) {
    for (;;) {
        if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
            return;
        }
        timestamp = millis();

        sensors_event_t accel, gyro, mag;
        accelerometer->getEvent(&accel);
        gyroscope->getEvent(&gyro);
        magnetometer->getEvent(&mag);

        cal.calibrate(mag);
        cal.calibrate(accel);
        cal.calibrate(gyro);

        gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
        gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
        gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

        filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

        if (counter++ <= POLL_RATE) {
            return;
        }
        counter = 0;

        imu.roll = filter.getRoll();
        imu.pitch = filter.getPitch();
        imu.heading = filter.getYaw();
    }
}

// Get PS4 data
void getJoystick() {
    //turbo
    if (PS4.Triangle()) {
        turbo = 1;
    }
    else {
        turbo = 0;
    }
    // forward
    if (PS4.R2Value() > 10 && PS4.L2Value() < 10) {
        mapY = map(PS4.R2Value(), 0, 255, 0, 100);
    }
    // backward
    else if (PS4.L2Value() > 10 && PS4.R2Value() < 10) {
        mapY = map(PS4.L2Value(), 0, 255, 0, -100);
    }
    // brake
    else {
        mapY = 0;
    }

    // manual
    bool cross = PS4.Cross();
    if (cross != lastCross) {
        if (cross) {
            manual = !manual;
        }
    }
    lastCross = cross;

    // kill switch
    bool square = PS4.Square();
    if (square != lastSquare) {
        if (square) {
            brake = !brake;
        }
    }
    lastSquare = square;
}

// Set PS4 colour & vibration
void setJoystick() {
    short min;
    if (distF  <= distFL) {
        if (distF <= distFR) {
            min = distF;
        }
        else {
            min = distFR;
        }
    }
    else {
        if (distFL <= distFR) {
            min = distFL;
        }
        else min = distFR;
    }

    if (min > 45) {
        min = 45;
    }
    short red = map(min, 0, 45, 255, 0);
    short green = 255 - red;

    PS4.setLed(red, green, 0);

    short rumbleL, rumbleR;
    if (distF <= distFL && distF <= distFR) {
        rumbleL = rumbleR = map(distF, 0, 60, 255, 0);
    }
    else {
        rumbleL = map(distFL, 0, 60, 255, 0);
        rumbleR = map(distFR, 0, 60, 255, 0);
    }
    
    PS4.setRumble(rumbleL, rumbleR);
    PS4.sendToController();
}

// Send motor values to 32U4
void sendSpeed(int speed = 0, int pos = 90, bool turbo = 0) {
    uint16_t sendSize = 0;
    motors.speed = speed;
    motors.pos = pos;
    motors.turbo = turbo;
    sendSize = myTransfer.txObj(motors, sendSize);
    myTransfer.sendData(sendSize);
}

// Receive data from 32U4
void recvData() {
    if (myTransfer.available()){
        uint16_t recSize = 0;
        recSize = myTransfer.rxObj(sonars, recSize);
        recSize = myTransfer.rxObj(mixedData, recSize);
        distF = sonars.distF;
        distFL = sonars.distFL;
        distFR = sonars.distFR;
        distL = mixedData.distanceL;
        distR = mixedData.distanceR;
    }
}

// Operator override for calculations
bool operator==(const SonarStruct lhs, const SonarStruct rhs) {
    if ((lhs.distF - rhs.distF < 2) && (lhs.distF - rhs.distF > -2)
        && (lhs.distFL - rhs.distFL < 2) && (lhs.distFL - rhs.distFL > -2)
        && (lhs.distFR - rhs.distFR < 2) && (lhs.distFR - rhs.distFR > -2))
            return 1;
    else return 0;
}

// Operator override for calculations
bool operator!=(const SonarStruct lhs, const SonarStruct rhs) {
    return !(lhs == rhs);
}

// Operator override for calculations
SonarStruct operator/(SonarStruct lhs, const int rhs) {
    lhs.distF = lhs.distF / rhs;
    lhs.distFL = lhs.distFL / rhs;
    lhs.distFR = lhs.distFR / rhs;
    return lhs;
}

// Modular piece of obstacle avoidance algorithm
void calc() {
    if ((distFL <= distFR) && (distL <= distR)) {
        if (mapY >= 0) {
            mapX = 90 + 90;
        }
        else {
            mapX = 90 - 90;
            if (fullAuto) {
                if (old.motors.speed > 0) {
                    sendSpeed();
                    delay(100);
                }
            }
        }
    }
    else if ((distFL >= distFR) && (distL >= distR)) {
        if (mapY >= 0) {
            mapX = 90 - 90;
        }
        else {
            mapX = 90 + 90;
            if (fullAuto) {
                if (old.motors.speed > 0) {
                    sendSpeed();
                    delay(100);
                }
            }
        }
    }
    else {
        distLFL = 5 * distL + 4 * distFL;
        distRFR = 5 * distR + 4 * distFR;
        if (distLFL >= distRFR) {
            if (mapY >= 0) {
                mapX = 90 - 90;
            }
            else {
                mapX = 90 + 90;
                if (fullAuto) {
                    if (old.motors.speed > 0) {
                        sendSpeed();
                        delay(100);
                    }
                }
            }
        }
        else {
            if (mapY <= 0) {
                mapX = 90 + 90;
            }
            else {
                mapX = 90 - 90;
                if (fullAuto) {
                    if (old.motors.speed > 0) {
                        sendSpeed();
                        delay(100);
                    }
                }
            }
        }
    }
}

// Obstacle avoidance algorithm
void calcDir() {
    if (sonars != old.sonars || ((mixedData.distanceL != old.mixedData.distanceL) || (mixedData.distanceR != old.mixedData.distanceR))) {
        if (distF >= 60 && (distFL >= 60 && distFR >= 60) && (distL >= 40 && distR >= 40)) {
            mapX = 90;
        }
        else if ((distF <= 60) && (((distFL < 60 || distFR < 60) && (distFL > 40 && distFR > 40)) || ((distL < 40 || distR < 40) && (distR > 25 && distL > 25)))) {
            calc();
        }
        else if (((distFL <= 40 || distFR <= 40) && (distFL > 10 && distFR > 10)) || ((distL <= 25 || distR <= 25) && (distL > 5 && distR > 5))) {
            calc();
        }
        else if ((distFL <= 10) || (distFR <= 10) || (distL <= 5) || (distR <= 5)) {
            if (fullAuto) {
                sendSpeed();
                delay(200);
                SonarStruct values1 = sonars;
                recvData();
                if (values1 != sonars) {
                    SonarStruct values2 = sonars;
                    delay(100);
                    recvData();
                    if (sonars != values2) {
                        sonars.distF = (sonars.distF + values1.distF + values2.distF) / 3;
                        sonars.distFL = (sonars.distFL + values1.distFL + values2.distFL) / 3;
                        sonars.distFR = (sonars.distFR + values1.distFR + values2.distFR) / 3;
                    }
                }

                if ((distFL < 10) || (distFR < 10) || (distL < 5) || (distR < 5)) {
                    mapY = -60;
                    mapX = 90;
                }
            }
        }
    }
    else {
        mapX = old.motors.pos;
    }
}

// Optional printing of debug data
void printDebug() {
    #ifdef DEBUG
        Serial.print("Sonar front: ");
        Serial.println(distF);
        Serial.print("Sonar front left: ");
        Serial.println(distFL);
        Serial.print("Sonar front right: ");
        Serial.println(distFR);
        Serial.print("Sonar left: ");
        Serial.println(distL);
        Serial.print("Sonar right: ");
        Serial.println(distR);
        Serial.println();
        Serial.print("Steering angle: ");
        Serial.println(mapX);
        Serial.print("Throttle: ");
        Serial.println(mapY);
        Serial.println();
    #endif
}

// Setup function for Blynk mode
void setupBlynk()
{
    Serial.begin(115200);
    Blynk.setDeviceName("Alset");
    Blynk.begin(auth);
    delay(300);
    for (;;)
        loopBlynk();
}

// Loop function for Blynk mode
void loopBlynk() {
    switchCon();
    Blynk.run();
    recvData();
    if (!manual) calcDir();
    if (!brake) {
        if (!manual && mapY == 0)
            sendSpeed(mapY, old.motors.pos);
        else sendSpeed(mapY, mapX);
    } else sendSpeed();
    old = {sonars, mixedData, motors, manual};
    printDebug();
}

// Setup function for PS4 mode
void setupPS4() {
    PS4.begin("00:de:ad:be:ef:00");
    //setupIMU();
    for (;;)
        loopPS4();
}

// Loop function for PS4 mode
void loopPS4() {
    switchCon();
    if (PS4.isConnected()) {
        getJoystick();
        recvData();

        if ((millis() - lastSend) > 30) {
            setJoystick();
            lastSend = millis();
        }
      
        if (manual) {
            mapX = map(PS4.LStickX(), -128, 127, 0, 180);
        }
        else {
            calcDir();
        }
      
        if (!brake) {
            sendSpeed(mapY, mapX, turbo);
            old = {sonars, mixedData, motors, manual};
        }
        else {
            sendSpeed();
        }
        
        printDebug();
    }
    else {
        sendSpeed(); // no arguments = brake
    }
}

// Interrupt handler
void switchMode() {
    updateCon = 1;
}

// Checks update flag and reboots into new mode
void switchCon() {
    if (updateCon) {
        updateCon = 0;
        EEPROM.put(0, !conMode);
        EEPROM.commit();
        ESP.restart();
    }
}

// Main setup function
void setup() {
    EEPROM.begin(8);
    delay(200);
    EEPROM.get(0, conMode);
    if (conMode > 1) {
        conMode = 1;
        EEPROM.put(0, conMode);
        EEPROM.commit();
    }

    Serial2.begin(115200);
    myTransfer.begin(Serial2);

    #ifdef DEBUG
        Serial.begin(115200);
    #endif

    pinMode(0, INPUT_PULLUP);
    attachInterrupt(0, switchMode, RISING);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, conMode); 

    if (conMode)
        setupBlynk();
    else setupPS4();
}

// Required for compilation
void loop() {}

// Blynk brake button callback
BLYNK_WRITE(V0) {
    brake = !param.asInt();
}

// Blynk joystick callback
BLYNK_WRITE(V1) {
    int posX = param[0].asInt();
    int posY = param[1].asInt();
    mapX = map(posX, -255, 255, 0, 180);
    mapY = map(posY, -255, 255, maxSpeed*-1, maxSpeed);
    if (posY == 0) mapY = 0;
}

// Blynk speed slider callback
BLYNK_WRITE(V2) {
    maxSpeed = param.asInt();
}

// Blynk autopilot button callback
BLYNK_WRITE(V3) {
    manual = !param.asInt();
}