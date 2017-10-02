#include <Pickupsystem.h>

PickupSystem::PickupSystem(uint8_t iPinEM) {
  pinEM = iPinEM;
  pinMode(pinEM, OUTPUT);
}

void PickupSystem::pick() {
  digitalWrite(pinEM, HIGH);
}
void PickupSystem::release() {
  digitalWrite(pinEM, LOW);
}

void PickupSystem::loop() {

}
