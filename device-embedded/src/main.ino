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
//#include <Chassis.h>
#include <ChassisWalking.h>
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
  RESUME_LAST,
  IGEL_TRAIN,
};

struct IgelState {
  ChassisState chassis;
  SonarState sonar;
  VisionState vision;
  PickupState pickup;
  bool stop = true;
  IgelJobState job;
} state;

Sonar *sonar;
ChassisWalking *chassis;
Vision *vision;
PickupSystem *pickupSystem;

// Plugins
#include <Api.h>

int lc = 0;
IgelJobState lastJobState;

void setup() {
  Serial.begin(9600);
  /*while ( ! Serial ) {
      delay( 1 );
  }*/

  Serial.println("IgelBot: Booting...");
  sonar = new Sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE);
  //chassis = new Chassis(MOTOR_RIGHT, MOTOR_LEFT, SERVO_STEER_C1, SERVO_STEER_C2);
  chassis = new ChassisWalking(SERVO_FRONT_RIGHT, SERVO_BACK_RIGHT, SERVO_FRONT_LEFT, SERVO_BACK_LEFT, SERVO_BACKBONE);
  vision = new Vision();
  pickupSystem = new PickupSystem(PICKUPSYSTEM_PIN);

  // Initialze web Api
  setupApi(WIFI_CS, WIFI_IRQ, WIFI_RST, WIFI_EN, WIFI_LISTEN_PORT);

  Serial.println("IgelBot: System started");

  setJobState(IGEL_SEARCH);
  //setJobState(IGEL_TRAIN);
  stop();
}

void loop() {
  if(!state.stop) {
    if (state.job == IGEL_TRAIN) {
      chassis->forward();
      chassis->loop(&state.chassis);
    } else {
      // Loop components
      sonar->loop(&state.sonar);
      chassis->loop(&state.chassis);
      vision->loop(&state.vision);
      pickupSystem->loop(&state.pickup);
      // Run Actions based on strategy
      strategy();
    }

  } else {
    chassis->stop();
    pickupSystem->release();
    if (state.job != IGEL_TRAIN and state.job != IGEL_SEARCH) {
      setJobState(IGEL_SEARCH);
    }
  }
  loopApi();
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
  Serial.println (state.job);
}

/* Run Component actions based on the current state */
/* Strategy consts */

void strategy() {
  // To be discussed: When should Sonar be activated (in which sates)
  if (state.job == IGEL_OBSTACLE && state.sonar.obstacelDistance > MIN_OBJECT_DISTANCE_CM) {
    Serial.println("Sonar: No more obstacle");
    setJobState(RESUME_LAST);
  }
  if (state.sonar.obstacelDistance <= MIN_OBJECT_DISTANCE_CM && state.job != IGEL_OBSTACLE) {
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
    Serial.print("Track (dist, dev):");
    Serial.print(state.vision.targetDistance);
    Serial.print(" , ");
    Serial.println(state.vision.targetDeviation);
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

#define STEER_INERTIA 200
int steerCount=0;
SteerDirection currentSteer;
SteerDirection dectedSteer;


// Tell the Chassis to steer with a certain intertia
void steerControler(SteerDirection steer) {
  if (steer != currentSteer) {
    if (dectedSteer == steer) {
      steerCount++;
    }
    if (steerCount > STEER_INERTIA) {
      chassis->steer(steer, 1);
      currentSteer = steer;
      steerCount=0;
      Serial.println(steer);
    }
    dectedSteer = steer;
  }
}

void exeJobTrack() {
  // Follow Target
  chassis->forward(100);
  // Navigae to Target
  if (state.vision.targetDeviation > TARGET_NAV_TOLARANCE) {
    steerControler(STEER_RIGHT);
  }
  if (state.vision.targetDeviation < TARGET_NAV_TOLARANCE*(-1)) {
    steerControler(STEER_LEFT);
  }
  if (state.vision.targetDeviation <= TARGET_NAV_TOLARANCE && state.vision.targetDeviation >= TARGET_NAV_TOLARANCE*(-1)) {
      steerControler(STEER_STRAIGHT);
  }
  // Target found (Deviation in Target and Traget below mimial Tolerance)
  if (state.vision.targetDistance > TARGET_DISTANCE_TOLARANCE && (
      state.vision.targetDeviation < TARGET_DEVIATION_TOLARANCE ||
      state.vision.targetDeviation > TARGET_DEVIATION_TOLARANCE*(-1))
  ) {
    Serial.println("Track: arrived at target");
    setJobState(IGEL_PICK);
    return;
  }
  // Target lost: too far right or too far lest or not in camara (targetDistance = -1)
  if (( state.vision.targetDeviation > TARGET_DEVIATION_TOLARANCE ||
      state.vision.targetDeviation < TARGET_DEVIATION_TOLARANCE*(-1)) ||
      state.vision.targetDistance < 0
  ) {
      Serial.println("Track: target lost");
      setJobState(IGEL_SEARCH);
      return;
  }
}

void exeJobPick() {
  Serial.println("Pickup Target");
  vision->reset();
  chassis->forward();
  delay(2000);
  chassis->down();
  pickupSystem->pick();
  delay(2000);
  setJobState(IGEL_GOHOME);
}

void exeJobGoHome() {
  // For now, just wait 2 seconds
  Serial.println("Go Home");
  chassis->up();
  chassis->steer(STEER_STRAIGHT, 0);
  chassis->forward();
  delay(2000);
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
