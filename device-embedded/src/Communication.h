#ifndef communication_h
#define communication_h

/*
  Use any framework that makes sense. On radar: ArduinoJson, aREST, Sming, ....
  The Communcaiton Class collects a stack of request from external and sends data to the cloud
  The Strategy method will analyze the stack of requests and decide what to do...
*/

#include "Arduino.h"
#include "Modular.h"
#include <ArduinoJson.h>

#define JSONBUFFERSIZE 500 // JSON Buffer Size

// Move to config later on
/* Waliking Pattern
  Prepare: http://arduinojson.org/assistant/
  {
    "speed" : 1000,
    "RF" : {
      "min" : 150,
      "max" : 600,
      "steps" : [[0,100], [1,0], [2,-100], [3,0]]
    },
    "LF" : {
      "min" : 150,
      "max" : 600,
      "steps" : [[4,100], [5,0], [6,-100], [7,0]]
    },
    "RF" : {
      "min" : 150,
      "max" : 600,
      "steps" : [[8,100], [9,0], [10,-100], [11,0]]
    },
    "LF" : {
      "min" : 150,
      "max" : 600,
      "steps" : [[12,100], [13,0], [14,-100], [15,0]]
    }
  }
*/
const char CW_DEFAULT_PATTERN[] = "{\"speed\":1000,\"RF\":{\"min\":150,\"max\":600,\"steps\":[[8,100],[9,0],[10,-100],[11,0]]},\"LF\":{\"min\":150,\"max\":600,\"steps\":[[12,100],[13,0],[14,-100],[15,0]]}}";

struct CommunicationState {
  bool hasRequests = false;
};

class Communication : public Component
{
  public:
    Communication();
    void loop(CommunicationState *state);
  protected:
    CommunicationState currentState;
};

#endif
