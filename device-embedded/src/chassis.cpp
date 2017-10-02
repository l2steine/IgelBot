
#include <Chassis.h>
#include <Adafruit_MotorShield.h>

Chassis::Chassis(uint8_t motorNumRight, uint8_t motorNumLeft) {
  motorShield = Adafruit_MotorShield();
  motorShield.begin();
  motorBackRight = motorShield.getMotor(motorNumRight);
  motorBackLeft = motorShield.getMotor(motorNumLeft);
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

void Chassis::forward(int speed);
void Chassis::backward();
void Chassis::backward(int speed);
void Chassis::right();
void Chassis::right(int angle);
void Chassis::left();
void Chassis::left(int angle);

void Chassis::stop() {
  motorBackLeft->run(RELEASE);
  motorBackRight->run(RELEASE);
  currentState.moving = false;
}
