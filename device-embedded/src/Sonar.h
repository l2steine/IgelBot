#ifndef sonar_h
#define sonar_h

#include <Arduino.h>
#include <Modular.h>
#include <NewPing.h>

struct SonarState {
  int obstacelDistance = -1;
};

class Sonar : public Component
{
  public:
    Sonar(uint8_t triggerPin, uint8_t echoPin, unsigned int maxDistance);
    void loop(SonarState *state);
    int scan();

  protected:
    NewPing *sonar;
};

#endif
