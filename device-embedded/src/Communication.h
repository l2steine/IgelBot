#ifndef communication_h
#define communication_h

#include <Arduino.h>
#include <Modular.h>
#include <WiFi101.h>
#include <WiFiConfig.h>

struct CommunicationState {
  int state = 0;
};

class Communication : public Component
{
  public:
    Communication(int8_t cs, int8_t irq, int8_t rst, int8_t en);
    void loop(CommunicationState *state);

  protected:

};

#endif
