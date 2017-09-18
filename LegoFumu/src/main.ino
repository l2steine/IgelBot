#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <NewPing.h>

Adafruit_MotorShield currentMotorShield = Adafruit_MotorShield();
Adafruit_DCMotor* left = currentMotorShield.getMotor(2);
Adafruit_DCMotor* right = currentMotorShield.getMotor(1);
#define TRIGGER_PIN 13
#define ECHO_PIN 12
#define MAX_DISTANCE 200
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int speed = 200;
int trim = 0;
int state = 0; // 0 = stop, 1 = walking
String apiBuffer = "";

void setup() {
  Serial.begin(9600);
  currentMotorShield.begin();
}

void loop() {
  delay(10);
  readSerial();
  scanSonar();
}

void scanSonar() {
  unsigned int uS = sonar.ping_cm();
  if (uS < 10 && state == 1) {
    executeTask("stop","");
  }
  if (uS >= 10 && state == 0) {
    executeTask("speed", (String)speed);
  }
}

void executeTask(String task, String value) {
   // Implemnation need to be done in the derived class for now
   Serial.println(task + ":" + value);
   if (task == "trim") {
     trim = value.toInt();
   }
   if (task == "speed") {
     speed = value.toInt();
     left->setSpeed(speed);
     right->setSpeed(speed+trim);
     left->run(FORWARD);
     right->run(FORWARD);
     state = 1;
   }

   if (task == "stop") {
     left->run(RELEASE);
     right->run(RELEASE);
     state = 0;
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
