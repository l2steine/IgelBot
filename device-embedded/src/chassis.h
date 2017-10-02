#ifndef chassis_h
#define chassis_h

#include <Arduino.h>
#include <Modular.h>
#include <Adafruit_MotorShield.h>

struct ChassisState {
  int speed = 200;
  bool moving = false;
  uint8_t angel = 0; // - = left, + = right
};

class Chassis : public Component
{
  public:
    Chassis(uint8_t motorNumRight, uint8_t motorNumLeft);
    ChassisState currentState;

    void loop(ChassisState *state);
    void forward();
    void forward(int speed);
    void backward();
    void backward(int speed);
    void right();
    void right(int angle);
    void left();
    void left(int angle);
    void stop();

  protected:
    Adafruit_MotorShield motorShield;
    Adafruit_DCMotor *motorBackRight;
    Adafruit_DCMotor *motorBackLeft;

};

#endif
