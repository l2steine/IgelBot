#include <Vision.h>

Vision::Vision() {
  pixy.init();
}

void Vision::loop(VisionState *state) {
  // Ansatz: Das Ziel ist gefunden, wenn y > VISION_MAX_Y undf state.x in einem definierten Toleranzbereich
  // Für den Igel ist der Toleranzbereich so zu setzen, dass die Schnecke noch vom Magent erfasst wird
  int n = pixy.getBlocks(1);
  if (n > 0 && pixy.blocks[0].signature == state->targetSignature) {

     x = (int)pixy.blocks[0].x; // Values from 0 to 319
     y = (int)pixy.blocks[0].y; // Values from 0 to 199

     state->targetDeviation = x - VISION_MAX_X/2;
     state->targetDistance = y;
     //Serial.println(state->targetDeviation);
  }
  else {
    state->targetDeviation = VISION_MAX_X/2;
    state->targetDistance = -1;
  }

}
