#include <Communication.h>
#include <ArduinoJson.h>

Communication::Communication() {
  Serial.print("Init Communication System ...");
  Serial.println(") [OK]");
}

void Communication::loop(CommunicationState *state) {
  state->hasRequests = currentState.hasRequests;
}

void apiSetWalkingPattern() {
  size_t bufferSize = 8*JSON_ARRAY_SIZE(2) + 2*JSON_ARRAY_SIZE(4) + 3*JSON_OBJECT_SIZE(3) + 130;
  DynamicJsonBuffer jsonBuffer(bufferSize);
  JsonObject& json = jsonBuffer.parseObject(CW_DEFAULT_PATTERN);
  // translate to
}
