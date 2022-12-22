#include <PS4Controller.h>
#include <ESP32Servo.h>

Servo servo;
Servo ESC;

const int servoPin = 12;
const int ESCPin = 13;

struct Controller {
  unsigned int lastUpdate;
  unsigned int R2Value;
  unsigned int L2Value;
  int LStickX;
  bool Square;
  bool Triangle;
  bool Cross;
  bool Circle;
  bool R1;
  bool L1;
  struct {
    bool Up;
    bool Down;
    bool Left;
    bool Right;
  } Dpad;
  bool L3;
} controller;

struct State {
  bool ignition;
  int gear;
  struct {
    bool up;
    bool down;
    bool left;
    bool right;
  } lights;
} state;

// throttle and brake from 0 to 255
// gear from 0 to 4
// net speed = speed * gear
struct Speed {
  int throttle;
  int brake;
  int gear;
  bool nitro;
  int net;
} speed;

const float gears[5] = {0.1, 0.25, 0.50, 0.75, 1};
const float nitroMultiplier = 1.4;

struct Motor {
  bool ignition;
  int power;
  int steering;
} motor;

const int neutral = 1500;
const int minSpeed = neutral + 70;
const int topSpeed = neutral + 150;
const int minSpeedRev = neutral - 150;
const int topSpeedRev = neutral - 250;

Controller getController() {
  Controller c;
  c.R2Value = PS4.R2Value();
  c.L2Value = PS4.L2Value();
  c.LStickX = PS4.LStickX();
  c.Square = PS4.Square();
  c.Triangle = PS4.Triangle();
  c.Cross = PS4.Cross();
  c.R1 = PS4.R1();
  c.L1 = PS4.L1();
  c.Dpad.Up = PS4.Up();
  c.Dpad.Down = PS4.Down();
  c.Dpad.Left = PS4.Left();
  c.Dpad.Right = PS4.Right();
  return c;
}

void controllerEvent()
{
  if(PS4.event.button_down.square)
    state.ignition = !state.ignition;
  if(PS4.event.button_down.r1)
    if(state.gear < 4)
      state.gear++;
  if(PS4.event.button_down.l1)
    if(state.gear > 0)
      state.gear--;
  if(PS4.event.button_down.ps)
    ESP.deepSleep(0);
  // TODO: handle dpad
}

Speed getSpeed() {
  Speed s;
  s.throttle = controller.R2Value;
  s.brake = controller.L2Value;
  s.nitro = controller.Triangle;
  s.gear = state.gear;

  s.net = int((s.throttle - s.brake) * gears[s.gear]);
  if (s.nitro) {
  s.net = s.net * nitroMultiplier;
}
if (s.net > 255) {
  s.net = 255;
} else if (s.net < -255) {
  s.net = -255;
}
s.net = map(s.net, -255, 255, -100, 100);

return s;
}

Motor getMotor() {
  Motor m;
  m.ignition = state.ignition;
  m.steering = controller.LStickX;
  m.steering = map(m.steering, -128, 127, 180, 0);
  if (speed.net > 0) {
    m.power = map(speed.net, 1, 100, minSpeed, topSpeed);
  } else if (speed.net < 0) {
    m.power = map(speed.net, -1, -100, minSpeedRev, topSpeedRev);
  } else {
    m.power = neutral;
  }
  return m;
}

void applyMotor() {
  if(motor.ignition)
    ESC.writeMicroseconds(motor.power);
  else ESC.writeMicroseconds(neutral);
  servo.write(motor.steering);  
}

void resetMotor() {
  ESC.writeMicroseconds(neutral);
  servo.write(90);
}

void printData() {
  Serial.print("power: ");
  Serial.println(motor.power);
  Serial.print("steering: ");
  Serial.println(motor.steering);
  Serial.print("ignition: ");
  Serial.println(motor.ignition);
  Serial.print("net speed: ");
  Serial.println(speed.net);
  Serial.print("throttle: ");
  Serial.println(speed.throttle);
  Serial.print("brake: ");
  Serial.println(speed.brake);
  Serial.print("gear: ");
  Serial.println(speed.gear);
  Serial.print("nitro: ");
  Serial.println(speed.nitro);
  Serial.println("\n");
}

void setup() {
  Serial.begin(115200);
  PS4.begin("00:45:E2:D2:4F:E6");
  while (!PS4.isConnected()) {}
  servo.attach(servoPin);
  ESC.attach(ESCPin);
  resetMotor();
  delay(1000);
  state = {false, 0, {1, 0, 0, 0}};
  PS4.attach(controllerEvent);
  PS4.setLed(0, 255, 0);
  PS4.sendToController();
}

void loop() {
  if (PS4.isConnected()) {
    controller = getController();
    speed = getSpeed();
    motor = getMotor();
    applyMotor();
    printData();
  } else resetMotor();
  delay(80);
}
