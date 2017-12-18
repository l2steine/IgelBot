
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
  startFrame[0] = 0;
  startFrame[1] = 100;
  startFrame[2] = 100;
  startFrame[3] = 0;
  legAmp[0] = 100;
  legAmp[1] = 100;
  legAmp[2] = 100;
  legAmp[3] = 100;
  legSpeed[0] = 1;
  legSpeed[1] = 1;
  legSpeed[2] = 1;
  legSpeed[3] = 1;
  legTrim[0] = 80;
  legTrim[1] = -15;
  legTrim[2] = 0;
  legTrim[3] = 15;
  currentSteer = STEER_STRAIGHT;
  reset();
  Serial.println("[OK]");
}

void ChassisWalking::setStartPosition(int leg, int frame) {

}

int ChassisWalking::getMiddlePos() {
  return (int)servomin + (servomax-servomin)*1.0/2;
}

void ChassisWalking::reset() {
  int middle = getMiddlePos();
  for (int s = 0; s < 4; s++) {
    legPos[s] = middle;
    legStarted[s] = false;
    pwm.setPWM(legServ[s], 0, legPos[s]);
  }
  legDirection[0] = -1; // Right side negative movement to move to front
  legDirection[1] = -1; // Right side negative movement to move to front
  legDirection[2] = 1;
  legDirection[3] = 1;
  //Serial.println(legSpeed[1]);
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

  //currentState.moving = true;
}
void ChassisWalking::backward(int speed) {
  currentState.speed = speed;
  backward();
}

void ChassisWalking::steer(SteerDirection direction, int angel) {
  if (currentSteer != direction) {
    currentSteer = direction;
    switch (direction) {
      case STEER_LEFT:
        startFrame[0] = 0;
        startFrame[1] = 100;
        startFrame[2] = 50;
        startFrame[3] = 150;
        reset();
        break;
      case STEER_RIGHT:
        startFrame[0] = 50;
        startFrame[1] = 150;
        startFrame[2] = 0;
        startFrame[3] = 100;
        reset();
        break;
      case STEER_STRAIGHT:
        startFrame[0] = 0;
        startFrame[1] = 100;
        startFrame[2] = 100;
        startFrame[3] = 0;
        reset();
        break;
    }
  }
}

void ChassisWalking::stop() {
  //ToDo: set all legs to postion of step 0
  //if (currentState.moving) {
    //reset();
    for (int s = 0; s < 4; s++) {
      pwm.setPWM(legServ[s], 0, 0);
    }
  //
  currentState.moving = false;
}

void ChassisWalking::down() {
  legDirection[0] = -1; // Right side negative movement to move to front
  legDirection[1] = -1; // Right side negative movement to move to front
  legDirection[2] = 1;
  legDirection[3] = 1;
  currentState.moving = false;
  int middle = getMiddlePos();
  for (int d = 0; d < downpos; d++ ) {
    for (int s = 0; s < 4; s++) {
      legPos[s] = middle + d * legDirection[s];
      pwm.setPWM(legServ[s], 0, legPos[s]);
      legStarted[s] = false;
      delay(10);
    }
  }
}

void ChassisWalking::up() {
  legDirection[0] = -1; // Right side negative movement to move to front
  legDirection[1] = -1; // Right side negative movement to move to front
  legDirection[2] = 1;
  legDirection[3] = 1;
  currentState.moving = false;
  int middle = getMiddlePos();
  for (int d = 0; d < downpos; d++ ) {
    for (int s = 0; s < 4; s++) {
      legPos[s] = middle + (downpos - d) * legDirection[s];
      pwm.setPWM(legServ[s], 0, legPos[s]);
      legStarted[s] = false;
      delay(10);
    }
  }
}

void ChassisWalking::loop(ChassisState *state) {
  // Drive each servo one at a time
  state->moving = currentState.moving;

  if (currentState.moving) {
    //Chekc if next frame is reached
    if (time + frameIntervall < micros()) {
      // check if next step is reached
      for (int s = 0; s < 4; s++) {
        //pwm.setPWM(legServ[s],0, legPos[s]);
        int trimPos = legPos[s] + legTrim[s];
        pwm.setPWM(legServ[s], 0, trimPos);
        if (frame >= startFrame[s] || legStarted[s] == true) {
          legStarted[s] = true;
          legPos[s] = (int)(legPos[s] + legDirection[s] * legSpeed[s]);
          if (legPos[s] > servomax*legAmp[s]*1.0/100 || legPos[s] < servomin*legAmp[s]*1.0/100) {
            legDirection[s] = legDirection[s]*-1;
          }
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
