#include <Arduino.h>

#if !defined (MOTOR_A1) || !defined (MOTOR_A2) || !defined (MOTOR_B1) || !defined (MOTOR_B2) || !defined (MOTOR_A_EN) || !defined (MOTOR_B_EN)
    #warning Motor pins not defined, using defaults
    #define MOTOR_A1 2
    #define MOTOR_A2 3
    #define MOTOR_B1 4
    #define MOTOR_B2 5
    #define MOTOR_A_EN 6
    #define MOTOR_B_EN 7
#endif

#if !defined (follow_line) && !defined (avoid_obstacles) && !defined (return_line) && !defined (impact_sensor) && !defined (comms) && !defined (imu) && !defined (leds) && !defined (autoOff) && !defined (serial_log) && !defined (ir_remote) && !defined (display)
    #warning No features enabled
#endif

#if follow_line
    #if !defined (MAKERLINE_AN) || !defined (MAX_SPEED) || !defined (INTERVAL)
        #define MAKERLINE_AN A0
        #define MAX_SPEED 255
        #define INTERVAL 10
    #endif
#endif

#if avoid_obstacles
    #include <Ultrasonic.h>
    #if !defined (L_TRIG) || !defined(L_ECHO) || !defined (R_TRIG) || !defined (R_ECHO) || !defined(F_TRIG) || !defined(F_ECHO)
        #define L_TRIG 33
        #define L_ECHO 32
        #define R_TRIG 35
        #define R_ECHO 34
        #define F_TRIG 37
        #define F_ECHO 30
    #endif
#endif

#if imu
    #include <Adafruit_Sensor_Calibration.h>
    #include <Adafruit_AHRS.h>
    Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
    #include "NXP_FXOS_FXAS.h"
    #if !defined (FILTER_UPDATE_RATE_HZ) || !defined (IMU_BUFFER)
        #define FILTER_UPDATE_RATE_HZ 100
        #define IMU_BUFFER 10
    #endif
#endif

#if ir_remote
    #ifndef IR_INPUT_PIN
        #define IR_INPUT_PIN 19
    #endif
    #define STR_HELPER(x) #x
    #define STR(x) STR_HELPER(x)
#endif

#if comms
    #include <SerialTransfer.h>
    #include <Servo.h>
#endif

#if display
    #include <LiquidCrystal_I2C.h>
#endif

class Robot {
    public:
        Robot()
        bool detectObstacle;
        void followLine;
    private:
        L298NX2 motors;
        Ultrasonic sonarL;
        Ultrasonic sonarR;
        Ultrasonic sonarF;

        int m_adcMakerLine = 0;
        int m_adcSetPoint = 0;
        int m_proportional = 0;
        int m_lastProportional = 0;
        int m_derivative = 0;
        int m_powerDifference = 0;
        int m_motorLeft = 0;
        int m_motorRight = 0;
        unsigned long m_currentMillis = 0;
        unsigned long m_previousMillis = 0;
        const int m_interval = 10;
}