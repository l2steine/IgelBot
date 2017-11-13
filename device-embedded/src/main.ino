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
#include <WiFi101.h>
#include <aREST.h>
#include <WiFiConfig.h>
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

Sonar *sonar;
ChassisWalking *chassis;
Vision *vision;
PickupSystem *pickupSystem;

// Declarations for WiFi API
WiFiServer* server;
WiFiClient client;
int status = WL_IDLE_STATUS;
aREST rest = aREST();
// Declare Functions for API
int start(String command);
int stop(String command);

int lc = 0;
IgelJobState lastJobState;

void setup() {
  Serial.begin(9600);
  while ( ! Serial ) {
      delay( 1 );
  }
  Serial.println("IgelBot: Booting...");
  sonar = new Sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE);
  //chassis = new Chassis(MOTOR_RIGHT, MOTOR_LEFT, SERVO_STEER_C1, SERVO_STEER_C2);
  chassis = new ChassisWalking(SERVO_FRONT_RIGHT, SERVO_FRONT_LEFT, SERVO_BACK_RIGHT, SERVO_BACK_LEFT, SERVO_BACKBONE);
  vision = new Vision();
  pickupSystem = new PickupSystem(PICKUPSYSTEM_PIN);
  // Setup the WiFi Connection
  WiFi.setPins(8,7,4,2);
  server = new WiFiServer(LISTEN_PORT);
  rest.function("start",start);
  rest.function("stop",stop);
  rest.set_id("IG1000");
  rest.set_name("Igel 1.0");
  // Connect to WiFi
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, password);
    // Wait 10 seconds for connection:
    delay(1000);
  }
  Serial.println("WiFi connected");
  server->begin();
  Serial.println("Web Server started");
  // Print the IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println("IgelBot: System started");
  setJobState(IGEL_SEARCH);
}

void loop() {
  delay(10);
  if(!state.stop) {
    // Loop components
    //sonar->loop(&state.sonar);
    chassis->forward();
    chassis->loop(&state.chassis);
    //vision->loop(&state.vision);
    //pickupSystem->loop(&state.pickup);
    // Run Actions based on strategy
    //strategy();
  } else {
    chassis->stop();
    pickupSystem->release();
    setJobState(IGEL_SEARCH);
  }
  if (!client) {
    client = server->available();
  }
  else if (client.available()) {
      rest.handle(client);
  }
}

/* main Action Start */
int start(String command) {
    state.stop = false;
    return 1;
}

/* main Action Start */
int stop(String command) {
    state.stop = true;
    return 1;
}

void setJobState(IgelJobState job) {
  //ToDo: Only allow valid state transitions
  if (job == RESUME_LAST) {
    state.job = lastJobState;
  } else {
    lastJobState = state.job;
    state.job = job;
  }
  //Serial.print("New Job: ");
  //Serial.println (job);
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
