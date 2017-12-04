
#include <ChassisWalking.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

ChassisWalking::ChassisWalking(uint8_t servoNumRightFront, uint8_t servoNumRightBack, uint8_t servoNumLeftFront,  uint8_t servoNumLeftBack, uint8_t servoNumBackbone) {
  Serial.print("Init Chassis... ");
  pwm = Adafruit_PWMServoDriver(0x40);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  legServ[0] = servoNumRightFront;
  legServ[1] = servoNumRightBack;
  legServ[2] = servoNumLeftFront;
  legServ[3] = servoNumLeftBack;
  servoBackbone = servoNumBackbone;
  // Set Default Walking Pattern
  legPos[0] = SERVOMIN+1;
  legPos[1] = SERVOMAX-1;
  legPos[2] = SERVOMIN+1;
  legPos[3] = SERVOMAX-1;
  legAmp[0] = 100;
  legAmp[1] = 100;
  legAmp[2] = 100;
  legAmp[3] = 100;
  legDirection[0] = -1; // Right side negative movement to move to front
  legDirection[1] = -1; // Right side negative movement to move to front
  legDirection[2] = 1;
  legDirection[3] = 1;
  legSpeed[0] = 1;
  legSpeed[1] = 1;
  legSpeed[2] = 1;
  legSpeed[3] = 1;
  // END DEBUG
  time = micros();
  Serial.println("[OK]");
}

void ChassisWalking::setStartPosition(int leg, int frame) {
  if ()
}

void ChassisWalking::reset() {
  time = micros();
  frame = 0;
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
  //ToDo: set all legs to postion of step 0
  currentState.moving = false;
  for (int s = 0; s < 4; s++) {
    pwm.setPWM(legServ[s], 0, 0);
  }
  time = micros();
}

void ChassisWalking::loop(ChassisState *state) {
  // Drive each servo one at a time
  state->moving = currentState.moving;

  if (currentState.moving) {
    //Chekc if next frame is reached
    if (time + frameIntervall < micros()) {
      // check if next step is reached
      for (int s = 0; s < 4; s++) {
        pwm.setPWM(legServ[s], 0, legPos[s]);
        legPos[s] = legPos[s] + legDirection[s] * legSpeed[s];
        //Serial.println(legPos[s]);
        if (legPos[s] > SERVOMAX*legAmp[s]/100 || legPos[s] < SERVOMIN*legAmp[s]/100) {
          legDirection[s] = legDirection[s]*-1;
        }
      }
      frame++;
      if (frame > frameNumber) {
        frame = 0;
      }
      time = micros();
    }
  }
}
