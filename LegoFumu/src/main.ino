/*
  This is the sourcecode of IgelBot Project.
  But also, it is an approach to design a robotic system inspired be the React Flux Pattern
  For now, this pattern ins not properly embedded into a framework, but rather developed
  following a set of concentions / rules:
  - A system consists of number of components
  - Components are reusabel mechatronic units
  - A component has
    - a state: struct {ComponentName}State
    - a number of Actions, that initiaite a behavoir in physical or logical operation (e.g. a movement, read a sensor signal)
    - a loop method, that executes the next step.
      The Loop is ment have a very low porcessing time in order to have short reaction time from signals of other component.
  - Only a Component is allowed to update its state (through actions or loop).
  - The main loop works like flux:
    1. Read Input form the Supersystem
    2. Set Actions, based on the current State (you could call this step the "Strategy" step)
    3. Loop each component
*/

#include <Arduino.h>
#include <Modular.h>
#include <Sonar.h>
#include <Chassis.h>
//#include <Strategy.h>

#define MOTOR_RIGHT 1
#define MOTOR_LEFT 2
#define SONAR_TRIGGER_PIN 13
#define SONAR_ECHO_PIN 12
#define SONAR_MAX_DISTANCE 200

Sonar *sonar;
Chassis *chassis;
//Strategy *strategy;
struct IgelState {
  ChassisState chassis;
  SonarState sonar;
  int targetDistance = 100;
  int targetDeviation = 0; // minus is left, plus is right
  bool stop = true;
} state;

String apiBuffer = "";

void setup() {
  Serial.begin(9600);
  Serial.println("IgelBot: Booting...");
  sonar = new Sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE);
  chassis = new Chassis(MOTOR_RIGHT, MOTOR_LEFT);
  Serial.println("IgelBot: System started");
  //strategy = new Strategy();
  //strategy->start();
}

void loop() {
  delay(10);
  readSerial();

  if(!state.stop) {
    // Run Actions based on strategy
    strategy();
    // Loop components
    sonar->loop(&state.sonar);
    chassis->loop(&state.chassis);
  }
}

/* main Action Start */
void start() {
    state.stop = false;
}

/* main Action Start */
void stop() {
    state.stop = true;
}

/* Run Component actions based on the current state */
void strategy() {
  if (state.sonar.obstacelDistance > 15 && state.chassis.moving == false) {
    Serial.println("Strategy: Run forward");
    chassis->forward();
  } else {
    Serial.println("Strategy: Stop");
    chassis->stop();
  }
}

void executeTask(String task, String value) {
   // Implemnation need to be done in the derived class for now
   Serial.println(task + ":" + value);
   if (task == "start") {
     start();
   }

   if (task == "stop") {
     stop();
   }
}

void readSerial() {
  while (Serial.available() > 0) {
    //workstate:'workstate',rate:'blinkrate'\n
    //workstate:1, rate:30\n

    if (Serial.available() > 0) {
         char c = Serial.read();
         apiBuffer += c;
         //Serial.println(apiBuffer);
        if (c == ';') {
          //Serial.println("Task revieced: " + apiBuffer);
          String order = apiBuffer.substring(0, apiBuffer.length() - 1);
          int j = order.indexOf(':');
          String task = order.substring(0, j);
          String value = order.substring(j+1);
          task.trim();
          value.trim();
          executeTask(task, value);
          apiBuffer = "";
        }
    }
  }
}
