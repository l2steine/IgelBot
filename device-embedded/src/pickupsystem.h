#ifndef pickupsystem_h
#define pickupsystem_h

#include "Arduino.h"
#include "Modular.h"

class PickupSystem : public Component
{
  public:
    PickupSystem(uint8_t pinEM);
    void loop();
    void pick();
    void release();
  protected:
    uint8_t pinEM;
};

#endif
