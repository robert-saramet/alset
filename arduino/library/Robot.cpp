#include <Arduino.h>
#include <Robot.h>

Robot::Robot() { 
    motors = L298NX2 (MOTOR_A_EN, MOTOR_A1, MOTOR_A2, MOTOR_B_EN, MOTOR_B1, MOTOR_B2);
    sonarL = Ultrasonic (L_TRIG, L_ECHO);
    sonarR = Ultrasonic (R_TRIG, R_ECHO);
    sonarF = Ultrasonic (F_TRIG, F_ECHO);

    if (comms) {
        // This connection is used for the joystick
        ESP_PORT.begin(4800);
        myTransfer.begin(ESP_PORT);
    }
}

Robot::detectObstacle() {
    if(sonarF.read() < 20){
            motors.backward();
            delay(80);
            return 0;
        }
        if (sonarL.read() < 30 || sonarR.read() < 30) {
            leftTrack = 1;
            robot.setSpeed(255);
            if (sonarL.read() < sonarR.read()) {
                while (sonarL.read() < 35) {
                    motors.forwardA();
                    motors.backwardB();
                }
            }
            else if (sonarL.read() > sonarR.read()) {
                while (sonarR.read() < 35) {
                    motors.forwardB();
                    motors.backwardA();
                }
            }
            else { // If both sensors return same value, to avoid getting stuck
                motors.backwardFor(300);
                motors.reset();
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
            robot.setSpeed(0);
        }
        else if (m_adcMakerLine > 972) { 
            // Cross line / intersection
            robot.setSpeedA(MAX_SPEED - 25);
            robot.setSpeedB(MAX_SPEED - 25);
        }
        else {
            m_proportional = m_adcMakerLine - m_adcSetPoint;
            derivative = m_proportional - m_lastProportional;
            m_lastProportional = m_proportional;

            m_powerDifference = (m_proportional * 1.5) + (derivative * 5);

            if (m_powerDifference > MAX_SPEED) {
                robot.forwardA();
                robot.backwardB();
                m_motorLeft = MAX_SPEED;
                if (m_powerDifference < 2 * MAX_SPEED) {
                    m_motorRight = m_powerDifference - MAX_SPEED;
                }
                else {
                    m_motorRight = MAX_SPEED;
                }
            }

            else if (m_powerDifference < -MAX_SPEED) {
                robot.backwardA();
                robot.forwardB();
                m_motorRight = MAX_SPEED;
                if (m_powerDifference > 2 * -MAX_SPEED) {
                    m_motorLeft = -MAX_SPEED - m_powerDifference;
                    }
                else {
                    m_motorLeft = MAX_SPEED;
                }
            }

            else if (m_powerDifference < MAX_SPEED  && m_powerDifference > -MAX_SPEED) {
                robot.forward();
                if (m_powerDifference > 0) {
                    m_motorLeft = MAX_SPEED;
                    m_motorRight = MAX_SPEED - m_powerDifference;
                }
                else {
                    m_motorLeft = MAX_SPEED - m_powerDifference;
                    m_motorRight = MAX_SPEED;
                }
            }

            robot.setSpeedA(m_motorLeft);
            robot.setSpeedB(m_motorRight);
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

    if (myTransfer.available()) {
        // Load packet from ESP8266
        myTransfer.rxObj(payload);

        // Values for motors
        mapX1 = payload.pos_x1;
        mapY1 = payload.pos_y1;
        bool btnOld1 = btn1;
        btn1 = payload.sw1;

        // Values for servo
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
            robot.stop();
        }
        else {
            // Set robot speed & direction
            if (mapY1 >= 0) {
                robot.forward();
            }
            else {
                robot.backward();
                mapY1 *= -1;
            }
            if (mapX1 >= 0) {
                robot.setSpeedB(mapY1);
                robot.setSpeedA(mapY1 - mapX1);
            }
            else {
                mapX1 *= -1;
                robot.setSpeedB(mapY1 - mapX1);
                robot.setSpeedA(mapY1);
            }
        }

        // Change servo position
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
            servo.write(m_pos);
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
