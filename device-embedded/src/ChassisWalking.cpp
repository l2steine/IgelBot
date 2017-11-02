
#include <ChassisWalking.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

ChassisWalking::ChassisWalking(uint8_t servoNumRightFront, uint8_t servoNumLeftFront, uint8_t servoNumRightBack, uint8_t servoNumLeftBack, uint8_t servoNumBackbone) {
  Serial.print("Init Chassis... ");
  pwm = Adafruit_PWMServoDriver();
  servoRF = servoNumRightFront;
  servoLF = servoNumLeftFront;
  servoRB = servoNumRightBack;
  servoLB = servoNumLeftBack;
  servoBackbone = servoNumBackbone;
  Serial.println("[OK]");
}

void ChassisWalking::forward() {
  currentState.moving = true;
}

void ChassisWalking::forward(int speed) {
  currentState.speed = speed;
  forward();
}
void ChassisWalking::backward() {

  currentState.moving = true;
}
void ChassisWalking::backward(int speed) {
  currentState.speed = speed;
  backward();
}

void ChassisWalking::steer(SteerDirection direction, int angel) {
  switch (direction) {
    case STEER_LEFT:

      break;
    case STEER_RIGHT:

      break;
    case STEER_STRAIGHT:

      break;
  }
}

void ChassisWalking::stop() {

  currentState.moving = false;
}

void ChassisWalking::loop(ChassisState *state) {
  // Drive each servo one at a time
  state->moving = currentState.moving;
  Serial.println("Walk");
  if (currentState.moving) {
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
      pwm.setPWM(servoRF, 0, pulselen);
    }
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
      pwm.setPWM(servoRF, 0, pulselen);
    }
  }
}
