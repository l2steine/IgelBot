#ifndef chassis_h
#define chassis_h

#include <Arduino.h>
#include <Modular.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)


struct ChassisState {
  int speed = 100;
  bool moving = false;
  uint8_t angel = 0; // - = left, + = right
};

enum SteerDirection { STEER_LEFT, STEER_RIGHT, STEER_STRAIGHT };

class ChassisWalking : public Component
{
  public:
    ChassisWalking(uint8_t servoNumRightFront, uint8_t servoNumLeftFront, uint8_t servoNumRightBack, uint8_t servoNumLeftBack, uint8_t servoNumBackbone);

    ChassisState currentState;
    void loop(ChassisState *state);
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
    Adafruit_PWMServoDriver pwm;
};

#endif
