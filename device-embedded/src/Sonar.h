#ifndef sonar_h
#define sonar_h

#include <Arduino.h>
#include <Modular.h>
#include <NewPing.h>

struct SonarState {
  int obstacelDistance = 9999;
};

class Sonar : public Component
{
  public:
    Sonar(uint8_t triggerPin, uint8_t echoPin, unsigned int maxDistance);
    void loop(SonarState *state);

  protected:
    NewPing *sonar;
};

#endif
