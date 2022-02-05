#include <Stepper.h>

const int stepsPerRevolution = 2048;
const int deg30 = stepsPerRevolution / 12;
const int deg15 = deg30 / 2;
const int warmup = 20;
int count = 1;
int dir = 1;

Stepper radarStepper(stepsPerRevolution, 10, 8, 11, 9);

void setup() {
    Serial.begin(115200);
}

void loop() {
    if (count++ >= 6) {
        dir *= -1;
        count = 1;
    }
    
    radarStepper.setSpeed(15);
    radarStepper.step(warmup * dir);
    radarStepper.setSpeed(20);
    radarStepper.step(warmup * dir);
    radarStepper.setSpeed(25);
    radarStepper.step((deg15 - 2 * warmup) * dir);

    delay(20);
}
