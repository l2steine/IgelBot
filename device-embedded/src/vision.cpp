#include <Vision.h>
#include "Pixy.h"
#include <SPI.h>

Vision::Vision() {
  Serial.print("Init Pixycam ...");
  myPixy.init();
  Serial.println("[OK]");
}

void Vision::reset() {
  currentState.targetDeviation = 0;
  currentState.targetDistance = -1;
  x=0;
  y=0;
}

void Vision::setTarget(int signature) {
  currentState.targetSignature = signature;
}

void Vision::loop(VisionState *state) {
  // Ansatz: Das Ziel ist gefunden, wenn y > VISION_MAX_Y undf state.x in einem definierten Toleranzbereich
  // Für den Igel ist der Toleranzbereich so zu setzen, dass die Schnecke noch vom Magent erfasst wird

  uint16_t j;
  uint16_t blocks = myPixy.getBlocks();
  bool tagetFound = false;
  if (blocks) {
    for (j=0; j<blocks; j++) {
      // Must support multiple Targest later on (e.g Snails and Playground border)
      if (myPixy.blocks[j].signature == currentState.targetSignature) {
        lostCount = 0;
        x = (int)myPixy.blocks[j].x; // Values from 0 to 319
        y = (int)myPixy.blocks[j].y; // Values from 0 to 199
        currentState.targetDeviation = x - VISION_MAX_X/2;
        currentState.targetDistance = y;
        tagetFound = true;
      }
    }
  }
  if (tagetFound == false) {
    lostCount++;
    if (lostCount > TARGET_LOST_COUNT) {
      currentState.targetDeviation = 0;
      currentState.targetDistance = -1;
    }
  }
  state->targetDeviation = currentState.targetDeviation;
  state->targetDistance = currentState.targetDistance;
  //Serial.print(state->targetDistance = state->targetDistance);
}
