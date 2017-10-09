#include <Pickupsystem.h>

PickupSystem::PickupSystem(uint8_t iPinEM) {
  Serial.print("Init Pickup System ...");
  pinEM = iPinEM;
  pinMode(pinEM, OUTPUT);
  digitalWrite(pinEM, LOW);
  Serial.print(" (run on Pin ");
  Serial.print(pinEM);
  Serial.println(") [OK]");
}

void PickupSystem::pick() {
  currentState.picking = true;
  digitalWrite(pinEM, HIGH);
  Serial.println("HIGH");
}
void PickupSystem::release() {
  currentState.picking = false;
  digitalWrite(pinEM, LOW);
  Serial.println("LOW");
}

void PickupSystem::loop(PickupState *state) {
  if (currentState.picking) {
    //digitalWrite(pinEM, HIGH);
    //Serial.println("HIGH");
  } else {
    //digitalWrite(pinEM, LOW);
    //Serial.println("LOW");
  }
  state->picking = currentState.picking;
}
