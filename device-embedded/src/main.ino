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
    - a loop method, that executes the next step. A Component loop must never use a timefunciton such as delay()
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
#include <Vision.h>
#include <Pickupsystem.h>
#include <Pins.h>
// VOICE


enum IgelJobState {
  IGEL_SEARCH,
  IGEL_TRACK,
  IGEL_PICK,
  IGEL_GOHOME,
  IGEL_DROP,
  IGEL_OBSTACLE,
  RESUME_LAST
};

struct IgelState {
  ChassisState chassis;
  SonarState sonar;
  VisionState vision;
  PickupState pickup;
  bool stop = true;
  IgelJobState job;
} state;

String apiBuffer = "";
Sonar *sonar;
Chassis *chassis;
Vision *vision;
PickupSystem *pickupSystem;

int lc = 0;
IgelJobState lastJobState;

void setup() {
  Serial.begin(9600);
  Serial.println("IgelBot: Booting...");
  sonar = new Sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE);
  chassis = new Chassis(MOTOR_RIGHT, MOTOR_LEFT, SERVO_STEER_C1, SERVO_STEER_C2);
  vision = new Vision();
  pickupSystem = new PickupSystem(PICKUPSYSTEM_PIN);
  Serial.println("IgelBot: System started");
  setJobState(IGEL_SEARCH);
  start();
}

void loop() {
  //delay(10);
  readSerial();
  if(!state.stop) {
    // Loop components
    sonar->loop(&state.sonar);
    chassis->loop(&state.chassis);
    vision->loop(&state.vision);
    pickupSystem->loop(&state.pickup);
    // Run Actions based on strategy
    strategy();
  } else {
    chassis->stop();
    pickupSystem->release();
    setJobState(IGEL_SEARCH);
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

void setJobState(IgelJobState job) {
  //ToDo: Only allow valid state transitions
  if (job == RESUME_LAST) {
    state.job = lastJobState;
  } else {
    lastJobState = state.job;
    state.job = job;
  }
  Serial.print("New Job: ");
  Serial.println (job);
}

/* Run Component actions based on the current state */
/* Strategy consts */

void strategy() {
  // To be discussed: When should Sonar be activated (in which sates)
  if (state.job == IGEL_OBSTACLE && state.sonar.obstacelDistance > MIN_OBJECT_DISTANCE_CM) {
    Serial.println("Sonar: No more obstacle");
    setJobState(RESUME_LAST);
  }
  if (state.sonar.obstacelDistance <= MIN_OBJECT_DISTANCE_CM && state.chassis.moving == true) {
    Serial.println("Sonar: Obstacle found");
    setJobState(IGEL_OBSTACLE);
    chassis->stop();
    return;
  }
  if (state.job == IGEL_SEARCH) { exeJobSearch(); }
  if (state.job == IGEL_TRACK)  { exeJobTrack();  }
  if (state.job == IGEL_PICK)   { exeJobPick();   }
  if (state.job == IGEL_GOHOME) { exeJobGoHome(); }
  if (state.job == IGEL_DROP)   { exeJobDrop();   }
  lc++;
  if (lc == 100) {
    /*Serial.println(state.job);s
    Serial.print("Track (dist, dev):");
    Serial.print(state.vision.targetDistance);
    Serial.print(" , ");*/
    lc = 0;
  }
}

void exeJobSearch() {
  // Wait for snail to appear
  chassis->forward(150);
  chassis->steer(STEER_LEFT, 255);
  if (state.vision.targetDistance > 0) {
    setJobState(IGEL_TRACK);
  }
}

void exeJobTrack() {
  // Follow Target
  chassis->forward(100);
  // Navigae to Target
  if (state.vision.targetDeviation > TARGET_NAV_TOLARANCE) {
      chassis->steer(STEER_RIGHT, state.vision.targetDeviation*TARGET_CONTROL_P);
  }
  if (state.vision.targetDeviation < TARGET_NAV_TOLARANCE*(-1)) {
      chassis->steer(STEER_LEFT, state.vision.targetDeviation*TARGET_CONTROL_P*(-1));
  }
  if (state.vision.targetDeviation >= TARGET_NAV_TOLARANCE && state.vision.targetDeviation <= TARGET_NAV_TOLARANCE) {
      chassis->steer(STEER_STRAIGHT, 0);
  }
  // Target found
  if (state.vision.targetDistance > TARGET_DISTANCE_TOLARANCE &&
      state.vision.targetDeviation < TARGET_DEVIATION_TOLARANCE &&
      state.vision.targetDeviation > TARGET_DEVIATION_TOLARANCE*(-1)
  ) {
    Serial.println("Track: target found");
    setJobState(IGEL_PICK);
  }
  // Target lost
  if ((state.vision.targetDistance > TARGET_DISTANCE_TOLARANCE &&
      state.vision.targetDeviation > TARGET_DEVIATION_TOLARANCE &&
      state.vision.targetDeviation < TARGET_DEVIATION_TOLARANCE*(-1)) ||
      state.vision.targetDistance < 0
  ) {
      Serial.println("Track: target lost");
      setJobState(IGEL_SEARCH);
  }
}

void exeJobPick() {
  Serial.println("Pickup Target");
  vision->reset();
  chassis->forward(65);
  delay(900);
  chassis->stop();
  pickupSystem->pick();
  delay(2000);
  setJobState(IGEL_GOHOME);
}

void exeJobGoHome() {
  // For now, just wait 2 seconds
  Serial.println("Go Home");
  chassis->steer(STEER_STRAIGHT, 0);
  chassis->forward(200);
  delay(1000);
  chassis->stop();
  delay(2000);
  setJobState(IGEL_DROP);
}

void exeJobDrop() {
  Serial.println("Drop Target");
  pickupSystem->release();
  chassis->steer(STEER_STRAIGHT, 0);
  chassis->backward(200);
  delay(2000);
  setJobState(IGEL_SEARCH);
}



void executeTask(String task, String value) {
   // Implemnation need to be done in the derived class for now
   Serial.println(task + ":" + value);
   if (task == "start") {
     start();
   }
   if (task == "steerL") {
     int val = value.toInt();
     chassis->steer(STEER_LEFT, val);
   }
   if (task == "steerR") {
     int val = value.toInt();
     chassis->steer(STEER_RIGHT, val);
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
