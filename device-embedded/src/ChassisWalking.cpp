
#include <ChassisWalking.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

ChassisWalking::ChassisWalking(uint8_t servoNumRightFront, uint8_t servoNumLeftFront, uint8_t servoNumRightBack, uint8_t servoNumLeftBack, uint8_t servoNumBackbone) {
  Serial.print("Init Chassis... ");
  pwm = Adafruit_PWMServoDriver(0x40);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  servoRF = servoNumRightFront;
  servoLF = servoNumLeftFront;
  servoRB = servoNumRightBack;
  servoLB = servoNumLeftBack;
  servoBackbone = servoNumBackbone;
  time = micros();
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

  if (currentState.moving) {
    //move if steptime is reached, vor now move all legs simoultanously
    if (time + stepTime < micros()) {
      pulselen += servoDirAll;
      Serial.println(pulselen);
      /*pwm.setPWM(servoRF, 0, pulselen);
      pwm.setPWM(servoLF, 0, pulselen);
      pwm.setPWM(servoRB, 0, pulselen);
      pwm.setPWM(servoLB, 0, pulselen);*/
      if (pulselen > SERVOMAX || pulselen < SERVOMIN) {
        servoDirAll = servoDirAll*(-1);
        Serial.println("Change Direction");
      }
      time = micros();
    }
  }
}
