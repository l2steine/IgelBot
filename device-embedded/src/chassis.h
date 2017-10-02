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

enum SteerDirection { STEER_LEFT, STEER_RIGHT, STEER_STRAIGHT };

class Chassis : public Component
{
  public:
    Chassis(uint8_t motorNumRight, uint8_t motorNumLeft, uint8_t pinC1, uint8_t pinC2);

    ChassisState currentState;
    uint8_t pinC1;
    uint8_t pinC2;
    void loop(ChassisState *state);
    void forward();
    void forward(int speed);
    void backward();
    void backward(int speed);
    void steer(SteerDirection direction, int angle);
    void stop();

  protected:
    Adafruit_MotorShield motorShield;
    Adafruit_DCMotor *motorBackRight;
    Adafruit_DCMotor *motorBackLeft;
};

#endif
