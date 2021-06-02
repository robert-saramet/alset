#include <Arduino.h>
#include <Robot.h>

Robot::Robot() { 
    m_motors = L298NX2 (MOTOR_A_EN, MOTOR_A1, MOTOR_A2, MOTOR_B_EN, MOTOR_B1, MOTOR_B2);
    m_sonarL = Ultrasonic (L_TRIG, L_ECHO);
    m_sonarR = Ultrasonic (R_TRIG, R_ECHO);
    m_sonarF = Ultrasonic (F_TRIG, F_ECHO);
    m_myTransfer = SerialTransfer;
    m_servo = Servo;

    if (comms) {
        // This connection is used for the joystick
        ESP_PORT.begin(4800);
        m_myTransfer.begin(ESP_PORT);
    }

    filter = Adafruit_Madgwick;
    cal = Adafruit_Sensor_Calibration_EEPROM;
    m_timestamp = millis();
}

Robot::detectObstacle() {
    if(m_sonarF.read() < 20){
            m_motors.backward();
            delay(80);
            return 0;
        }
        if (m_sonarL.read() < 30 || m_sonarR.read() < 30) {
            leftTrack = 1;
            robot.setSpeed(255);
            if (m_sonarL.read() < m_sonarR.read()) {
                while (m_sonarL.read() < 35) {
                    m_motors.forwardA();
                    m_motors.backwardB();
                }
            }
            else if (m_sonarL.read() > m_sonarR.read()) {
                while (m_sonarR.read() < 35) {
                    m_motors.forwardB();
                    m_motors.backwardA();
                }
            }
            else { // If both sensors return same value, to avoid getting stuck
                m_motors.backwardFor(300);
                m_motors.reset();
            }
            return 0;
        }
}

Robot::followLine() {
    m_currentMillis = millis();
    if (m_currentMillis - m_previousMillis > m_interval) {
        m_previousMillis = m_currentMillis;

        m_adcMakerLine = analogRead(MAKERLINE_AN);

        if (m_adcMakerLine < 103) {
            // Outside line
            m_motors.setSpeed(0);
        }
        else if (m_adcMakerLine > 972) { 
            // Cross line / intersection
            m_motors.setSpeedA(MAX_SPEED - 25);
            m_motors.setSpeedB(MAX_SPEED - 25);
        }
        else {
            m_proportional = m_adcMakerLine - m_adcSetPoint;
            derivative = m_proportional - m_lastProportional;
            m_lastProportional = m_proportional;

            m_powerDifference = (m_proportional * 1.5) + (derivative * 5);

            if (m_powerDifference > MAX_SPEED) {
                m_motors.forwardA();
                m_motors.backwardB();
                m_motorLeft = MAX_SPEED;
                if (m_powerDifference < 2 * MAX_SPEED) {
                    m_motorRight = m_powerDifference - MAX_SPEED;
                }
                else {
                    m_motorRight = MAX_SPEED;
                }
            }

            else if (m_powerDifference < -MAX_SPEED) {
                m_motors.backwardA();
                m_motors.forwardB();
                m_motorRight = MAX_SPEED;
                if (m_powerDifference > 2 * -MAX_SPEED) {
                    m_motorLeft = -MAX_SPEED - m_powerDifference;
                    }
                else {
                    m_motorLeft = MAX_SPEED;
                }
            }

            else if (m_powerDifference < MAX_SPEED  && m_powerDifference > -MAX_SPEED) {
                m_motors.forward();
                if (m_powerDifference > 0) {
                    m_motorLeft = MAX_SPEED;
                    m_motorRight = MAX_SPEED - m_powerDifference;
                }
                else {
                    m_motorLeft = MAX_SPEED - m_powerDifference;
                    m_motorRight = MAX_SPEED;
                }
            }

            m_motors.setSpeedA(m_motorLeft);
            m_motors.setSpeedB(m_motorRight);
        }
    }
}

Robot::getJoystick() {
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

    if (m_myTransfer.available()) {
        // Load packet from ESP8266
        m_myTransfer.rxObj(payload);

        // Values for m_motors
        mapX1 = payload.pos_x1;
        mapY1 = payload.pos_y1;
        bool btnOld1 = btn1;
        btn1 = payload.sw1;

        // Values for m_servo
        mapX2 = payload.pos_x2;
        mapY2 = payload.pos_y2;
        bool btnOld2 = btn2;
        btn2 = payload.sw2;

        // Check m_brake button toggle
        if (btn1 != btnOld1) {
            if (btn1) {
                m_brake = !m_brake;
            }
        }

        if (m_brake) { // Press joystick button to stop robot
            m_motors.stop();
        }
        else {
            // Set robot speed & direction
            if (mapY1 >= 0) {
                m_motors.forward();
            }
            else {
                m_motors.backward();
                mapY1 *= -1;
            }
            if (mapX1 >= 0) {
                m_motors.setSpeedB(mapY1);
                m_motors.setSpeedA(mapY1 - mapX1);
            }
            else {
                mapX1 *= -1;
                m_motors.setSpeedB(mapY1 - mapX1);
                m_motors.setSpeedA(mapY1);
            }
        }m_motors
        // Change m_servo position
        if (mapX2 > 100 || mapX2 < -100) {
            if (mapX2 < 0) {
                if (m_pos >= 2)
                {
                    m_pos -= 2;
                }
            }
            else {
                if (m_pos <= 168) {
                    m_pos += 2;
                }
            }
            m_servo.write(m_pos);
        }

        // Check laser button toggle
        if (btn2 != btnOld2) {
            if (btn2) {
                m_pwr = !m_pwr;
            }
            if (m_pwr) {
                digitalWrite(laser, HIGH);
            }
            else {
                digitalWrite(laser, LOW);
            }
        }
    }
}

Robot::getAngle(char angle = 'z') {
    float gx, gy, gz;
    static uint8_t counter = 0;

    if ((millis() - m_timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
        return;
    }

    m_timestamp = millis();

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

    switch (angle) {
        case 'X':
        case 'x':
            return roll;
        case 'Y':
        case 'y':
            return pitch;
        case 'Z':
        case 'z':
            return heading;
    }
}
