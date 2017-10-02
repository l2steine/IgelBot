#ifndef Strategy_h
#define Strategy_h

#include "Arduino.h"
#include "Modular.h"
#include <Sonar.h>
#include <Chassis.h>

// This construct must be updated later on


class Strategy : public Component
{
  public:
    Strategy();
    void start();
    void stop();
    //void loop(IgelState *state);

  protected:

};

#endif
