#include <Vision.h>

Vision::Vision() {
  pixy.init();
}

void Vision::loop(VisionState *state) {

  int n = pixy.getBlocks(1);
  if (pixy.blocks[0].signature == state->targetSignature) {

     x = (int)pixy.blocks[0].x; // Values from 0 to 319
     y = (int)pixy.blocks[0].y; // Values from 0 to 199

     state->targetDeviation = x - VISION_MAX_X/2;
     state->targetDistance = y;
     Serial.println(state->targetDeviation);
  }
  else {
    state->targetDeviation = VISION_MAX_X/2;
    state->targetDistance = -1;
  }

}
