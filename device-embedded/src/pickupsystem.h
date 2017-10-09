#ifndef pickupsystem_h
#define pickupsystem_h

#include "Arduino.h"
#include "Modular.h"

struct PickupState {
  bool loaded = false;
  bool picking = false;
};

class PickupSystem : public Component
{
  public:
    PickupSystem(int iPinEM);
    void loop(PickupState *state);
    void pick();
    void release();
  protected:
    int pinEM;
    PickupState currentState;
};

#endif
