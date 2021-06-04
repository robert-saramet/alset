#include <L298NX2.h>

#define motorA1 2
#define motorA2 3
#define motorB1 5
#define motorB2 4
#define motorA_EN 6
#define motorB_EN 7
#define STBY 9

L298NX2 robot(motorB_EN, motorB1, motorB2, motorA_EN, motorA1, motorA2); //initialize motor driver

// Comment this out to disable prints and save space
//#define BLYNK_PRINT Serial


#include <BlynkSimpleSerialBLE.h>

char auth[] = "B3FFB419A40961239E202763E131A519C92915D048A32690A9E71F06A5E85CEB72ED63045F2791BAD5EF60CFD990AE85";

// Called on every app widget update
BLYNK_WRITE(V0)
{
  // Assigning incoming value from pin V0 to a variable
  bool pinValue = param.asInt(); 
  if (pinValue) {
    digitalWrite(STBY, HIGH);
  }
  else {
    robot.stop();
    digitalWrite(STBY, LOW);
  }
}


BLYNK_WRITE(V1) {
  int mapX = param[0].asInt();
  int mapY = param[1].asInt();

  if (mapY == 0 && mapX == 0) {
    robot.stop();
  }

  if (mapY >= 0) {
      robot.forward();
  }
  else {
        robot.backward();
        mapY *= -1;
  }
  
  if (mapX >= 0) {
        robot.setSpeedB(mapY);
        robot.setSpeedA(mapY - mapX);
  }
  else {
      mapX *= -1;
      robot.setSpeedB(mapY - mapX);
      robot.setSpeedA(mapY);
  }
}


void setup()
{
  // Debug console
  Serial.begin(9600);

  pinMode(STBY, OUTPUT);

  Serial2.begin(9600);
  Blynk.begin(Serial2, auth);

  Serial.println("Waiting for connections...");
  robot.setSpeed(255);
}

void loop()
{
  Blynk.run();
}
