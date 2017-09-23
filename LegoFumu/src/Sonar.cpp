#include "Sonar.h"
#include <NewPing.h>

Sonar::Sonar(uint8_t triggerPin, uint8_t echoPin, uint8_t maxDistance) {
  NewPing sonar(triggerPin, echoPin, maxDistance);
}

void Sonar::scan() {
  unsigned int uS = sonar.ping_cm();
  if (uS < 10) {
    //
  }
  if (uS >= 10) {
    //
  }
}

Message Sonar::componentLoop() {
  this->scan();
}
