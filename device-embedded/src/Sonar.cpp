#include <Sonar.h>
#include <NewPing.h>

Sonar::Sonar(uint8_t triggerPin, uint8_t echoPin, unsigned int maxDistance) {
    sonar = new NewPing(triggerPin, echoPin, maxDistance);
}

void Sonar::loop(SonarState *state) {
  unsigned int uS = sonar->ping_cm();
  state->obstacelDistance = uS;
}
