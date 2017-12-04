#ifndef chassis_h
#define chassis_h

#include <Arduino.h>
#include <Modular.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

#define SERVOMIN  200  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  320  // this is the 'maximum' pulse length count (out of 4096)
#define STEPS_PER_MEASURE 16

struct ChassisState {
  int speed = 100;
  bool moving = false;
  uint8_t angel = 0; // - = left, + = right
};

struct WalikingPattern {

};

enum SteerDirection { STEER_LEFT, STEER_RIGHT, STEER_STRAIGHT };

class ChassisWalking : public Component
{
  public:
    ChassisWalking(uint8_t servoNumRightFront, uint8_t servoNumLeftFront, uint8_t servoNumRightBack, uint8_t servoNumLeftBack, uint8_t servoNumBackbone);

    ChassisState currentState;
    void loop(ChassisState *state);
    void reset();
    void forward();
    void forward(int speed);
    void backward();
    void backward(int speed);
    void steer(SteerDirection direction, int angle);
    void stop();
    void setLegPosition(int leg, int frame);
    int legPos[4] = {};
    int legAmp[4] = {}; // Ignored fro now
    int legSpeed[4] = {};
    int legDirection[4] = {};
    int frameIntervall = 3000; // frameSize for interpolation in microseconds
    int frameNumber = 120; //stepSize in microseconds
  protected:
    int legServ[4] = {};
    Adafruit_PWMServoDriver pwm;
    uint8_t servoBackbone;
    unsigned long time;
    int frame = 0;

};

#endif
