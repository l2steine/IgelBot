#include <Sonar.h>
#include <NewPing.h>

Sonar::Sonar(uint8_t triggerPin, uint8_t echoPin, unsigned int maxDistance) {
    sonar = new NewPing(triggerPin, echoPin, maxDistance);
}

int Sonar::scan() {
  unsigned int uS = sonar->ping_cm();
  if (uS < 10) {
    //
  }
  if (uS >= 10) {
    //
  }
}

void Sonar::loop(SonarState *state) {
  unsigned int uS = sonar->ping_cm();
  state->obstacelDistance = uS;
}
