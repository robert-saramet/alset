#include "SerialTransfer.h"
#include "PS4Controller.h"
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

#undef DEBUG

SerialTransfer myTransfer;

struct __attribute__((__packed__)) MotorStruct {
    int8_t speed;
    uint8_t pos;
    bool turbo;
} motors;

struct __attribute__((__packed__)) SonarStruct {
    uint8_t distF;
    uint8_t distFL;
    uint8_t distFR;
} sonars;

struct __attribute__((__packed__)) MixedStruct {
    uint8_t distanceL;
    uint8_t distanceR;
    double lat;
    double lng;
    double speed;
    char dir;
} mixedData;

struct __attribute__((__packed__)) GyroStruct {
    float roll;
    float pitch;
    float heading;
} imu;    

struct OldData {
    struct SonarStruct sonars;
    struct MixedStruct mixedData;
    struct MotorStruct motors;
    bool manual;
} old {{60, 60, 60}, {40, 40, 0, 0, 0, 'L'}, {0, 90, 0}, 1};

bool turbo = 0;
bool manual = 0;
bool lastCross = 0;
bool fullAuto = 0;

short distF;
short distFL;
short distFR;
short distL;
short distR;

long distLFL;
long distRFR;

int mapX = 90;
int mapY = 0;

float gx, gy, gz;
short counter = 0;

long lastSend = millis();

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "NXP_FXOS_FXAS.h"

Adafruit_NXPSensorFusion filter;

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define POLL_RATE 10

uint32_t timestamp;

TaskHandle_t GyroTask;

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

void sendSpeed(int speed = 0, int pos = 90, bool turbo = 0) {
    uint16_t sendSize = 0;
    motors.speed = speed;
    motors.pos = pos;
    motors.turbo = turbo;
    sendSize = myTransfer.txObj(motors, sendSize);
    myTransfer.sendData(sendSize);
}

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
      
    bool cross = PS4.Cross();
    if (cross != lastCross) {
        if (cross) {
            manual = !manual;
        }
    }
    lastCross = cross;
}

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

bool operator==(const SonarStruct lhs, const SonarStruct rhs) {
    if ((lhs.distF - rhs.distF < 2) && (lhs.distF - rhs.distF > -2)
        && (lhs.distFL - rhs.distFL < 2) && (lhs.distFL - rhs.distFL > -2)
        && (lhs.distFR - rhs.distFR < 2) && (lhs.distFR - rhs.distFR > -2))
            return 1;
    else return 0;
}

bool operator!=(const SonarStruct lhs, const SonarStruct rhs) {
    return !(lhs == rhs);
}

SonarStruct operator/(SonarStruct lhs, const int rhs) {
    lhs.distF = lhs.distF / rhs;
    lhs.distFL = lhs.distFL / rhs;
    lhs.distFR = lhs.distFR / rhs;
    return lhs;
}

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
        Serial.print("Roll: ");
        Serial.println(imu.roll);
        Serial.print("Pitch: ");
        Serial.println(imu.pitch)
        Serial.print("Heading: ");
        Serial.println(imu.heading);
        Serial.println();
    #endif
}

void setup()
{
    #ifdef DEBUG
        Serial.begin(115200);
    #endif

    PS4.begin("00:de:ad:be:ef:00");
    Serial2.begin(115200);
    myTransfer.begin(Serial2);

    setupIMU();
}

void loop()
{
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
      
        sendSpeed(mapY, mapX, turbo);
        old = {sonars, mixedData, motors, manual};
        
        printDebug();
    }
    else {
        sendSpeed(); // no arguments = brake
    }
}
