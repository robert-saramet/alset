#include <Arduino.h>
#include <Robot.h>

Robot::Robot() { 
    motors = L298NX2 (MOTOR_A_EN, MOTOR_A1, MOTOR_A2, MOTOR_B_EN, MOTOR_B1, MOTOR_B2);
    sonarL = Ultrasonic (L_TRIG, L_ECHO);
    sonarR = Ultrasonic (R_TRIG, R_ECHO);
    sonarF = Ultrasonic (F_TRIG, F_ECHO);
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