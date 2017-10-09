
#include <Chassis.h>
#include <Adafruit_MotorShield.h>

Chassis::Chassis(uint8_t motorNumRight, uint8_t motorNumLeft, uint8_t iPinC1, uint8_t iPinC2) {
  Serial.print("Init Chassis... ");
  motorShield = Adafruit_MotorShield();
  motorShield.begin();
  motorBackRight = motorShield.getMotor(motorNumRight);
  motorBackLeft = motorShield.getMotor(motorNumLeft);
  pinC1 = iPinC1;
  pinC2 = iPinC2;
  Serial.println("[OK]");
}

void Chassis::loop(ChassisState *state) {
  state->moving = currentState.moving;
}

void Chassis::forward() {
  motorBackLeft->setSpeed(currentState.speed);
  motorBackRight->setSpeed(currentState.speed);
  motorBackLeft->run(FORWARD);
  motorBackRight->run(FORWARD);
  currentState.moving = true;
}

void Chassis::forward(int speed) {
  currentState.speed = speed;
  forward();
}
void Chassis::backward() {}
void Chassis::backward(int speed) {}

void Chassis::steer(SteerDirection direction, int angel) {
  switch (direction) {
    case STEER_LEFT:
      analogWrite(pinC1, 0);
      analogWrite(pinC2, angel);
      break;
    case STEER_RIGHT:
      analogWrite(pinC1, angel);
      analogWrite(pinC2, 0);
      break;
    case STEER_STRAIGHT:
      analogWrite(pinC1, 0);
      analogWrite(pinC2, 0);
      break;
  }
}

void Chassis::stop() {
  motorBackLeft->run(RELEASE);
  motorBackRight->run(RELEASE);
  currentState.moving = false;
}
