#ifndef chassis_h
#define chassis_h

#include <Arduino.h>
#include <Modular.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

#define SERVOMIN  0  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  400  // this is the 'maximum' pulse length count (out of 4096)
#define STEPS_PER_MEASURE 16

struct ChassisState {
  int speed = 100;
  bool moving = false;
  uint8_t angel = 0; // - = left, + = right
};
/*
// One foot after the other
//                              0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
int rf [STEPS_PER_MEASURE] = { 50, 72, 25, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 }; // Right Front
int lf [STEPS_PER_MEASURE] = { 50, 50, 50, 50, 50, 75, 25, 50, 50, 50, 50, 50, 50, 50, 50, 50 }; // Left Front
int rb [STEPS_PER_MEASURE] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 75, 25, 50, 50, 50, 50, 50 }; // Right Back
int lb [STEPS_PER_MEASURE] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 75, 25, 50 }; // Left Back

// Stand
//                              0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
int rf [STEPS_PER_MEASURE] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 }; // Right Front
int lf [STEPS_PER_MEASURE] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 }; // Left Front
int rb [STEPS_PER_MEASURE] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 }; // Right Back
int lb [STEPS_PER_MEASURE] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 }; // Left Back
*/
struct WalikingPattern {
  int period = 5000;
  //                              0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
  int rf [STEPS_PER_MEASURE] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 }; // Right Front
  int lf [STEPS_PER_MEASURE] = {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }; // Left Front
  int rb [STEPS_PER_MEASURE] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100 }; // Right Back
  int lb [STEPS_PER_MEASURE] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100 }; // Left Back
  int bb [STEPS_PER_MEASURE] = { }; // Backbone
  int rfPwm0 = SERVOMIN;
  int lfPwm0 = SERVOMIN;
  int rbPwm0 = SERVOMIN;
  int lbPwm0 = SERVOMIN;
  int rfPwm100 = SERVOMAX-SERVOMIN;
  int lfPwm100 = SERVOMAX-SERVOMIN;
  int rbPwm100 = SERVOMAX-SERVOMIN;
  int lbPwm100 = SERVOMAX-SERVOMIN;
};

enum SteerDirection { STEER_LEFT, STEER_RIGHT, STEER_STRAIGHT };

class ChassisWalking : public Component
{
  public:
    ChassisWalking(uint8_t servoNumRightFront, uint8_t servoNumLeftFront, uint8_t servoNumRightBack, uint8_t servoNumLeftBack, uint8_t servoNumBackbone);

    ChassisState currentState;
    void loop(ChassisState *state);
    void setWalkingPattern(WalikingPattern pattern);
    void forward();
    void forward(int speed);
    void backward();
    void backward(int speed);
    void steer(SteerDirection direction, int angle);
    void stop();

  protected:
    uint8_t servoRF;
    uint8_t servoLF;
    uint8_t servoRB;
    uint8_t servoLB;
    uint8_t servoBackbone;
    WalikingPattern currentWalkingPattern;
    Adafruit_PWMServoDriver pwm;
    int servoDirAll = 1;
    int pulselen = SERVOMIN;
    unsigned long time;
    unsigned long stepTime;
    int frameSize = 1000; // frameSize for interpolation in microseconds
    int stepSize; //stepSize in microseconds
    int step = 0;
    double slopeRF = 0;
    double slopeLF = 0;
    double slopeRB = 0;
    double slopeLB = 0;
    double calculateSlope(int steps[], int step);
};

#endif
