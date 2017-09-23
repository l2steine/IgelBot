#ifndef sonar_h
#define sonar_h

#include "Arduino.h"
#include "Modular.h"
#include <NewPing.h>

class Sonar : public Component
{
  public:
    Sonar(uint8_t triggerPin, uint8_t echoPin, uint8_t maxDistance);
    Message componentLoop();
  protected:
    void scan();
    NewPing sonar;
};

#endif
