
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
  // DEBUG
  setWalkingPattern(currentWalkingPattern);
  // END DEBUG
  time = micros();
  stepTime = micros();
  Serial.println("[OK]");
}

void ChassisWalking::setWalkingPattern(WalikingPattern pattern) {
  currentWalkingPattern = pattern;
  stepSize = currentWalkingPattern.period/STEPS_PER_MEASURE*1000;
  Serial.print("Step Size: ");
  Serial.println(stepSize);
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
  pwm.setPWM(servoRF, 0, 0);
  pwm.setPWM(servoLF, 0, 0);
  pwm.setPWM(servoRB, 0, 0);
  pwm.setPWM(servoLB, 0, 0);
  time = micros();
}

double ChassisWalking::calculateSlope(int steps[], int step) {
  // this last defined step
  int s1 = step;
  int s2 = step+1;
  if (s2 >= STEPS_PER_MEASURE) {
    s2 = 0;
  }
  double slope = 0;
  slope = (steps[s2]-steps[s1])*1.0/stepSize;
  return slope;
}

void ChassisWalking::loop(ChassisState *state) {
  // Drive each servo one at a time
  state->moving = currentState.moving;

  if (currentState.moving) {
    //Chekc if next frame is reached
    if (time + frameSize < micros()) {
      // check if next step is reached
      if (stepTime + stepSize < micros()) {
        // calculate new slope for each leg
        step++;
        if (step >= STEPS_PER_MEASURE) {
          step = 0;
        }
        // interplaote slope between servoMin and ServoMax
        slopeRF = calculateSlope(currentWalkingPattern.rf, step);
        slopeLF = calculateSlope(currentWalkingPattern.lf, step);
        slopeRB = calculateSlope(currentWalkingPattern.rb, step);
        slopeLB = calculateSlope(currentWalkingPattern.lb, step);
        stepTime = time;
        //Serial.print("Step: ");
        //Serial.println(step);
      }
      int dt = time - stepTime;
      pwm.setPWM(servoRF, 0, currentWalkingPattern.rfPwm0 + dt * (slopeRF/100) * currentWalkingPattern.rfPwm100);
      pwm.setPWM(servoLF, 0, currentWalkingPattern.lfPwm0 + dt * (slopeLF/100) * currentWalkingPattern.lfPwm100);
      pwm.setPWM(servoRB, 0, currentWalkingPattern.rbPwm0 + dt * (slopeRB/100) * currentWalkingPattern.rbPwm100);
      pwm.setPWM(servoLB, 0, currentWalkingPattern.lbPwm0 + dt * (slopeLB/100) * currentWalkingPattern.lbPwm100);
      time = micros();
    }
  }
}
