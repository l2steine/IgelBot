#include <Arduino.h>

#include <Adafruit_MotorShield.h>

Adafruit_MotorShield currentMotorShield = Adafruit_MotorShield();
Adafruit_DCMotor* left = currentMotorShield.getMotor(2);
Adafruit_DCMotor* right = currentMotorShield.getMotor(1);
int speed = 0;
int trim = -20;
String apiBuffer = "";

void setup() {
  Serial.begin(9600);
  currentMotorShield.begin();
}

void loop() {
  delay(10);
  readSerial();
}

void handleSerialInstruction(String task, String value) {
   // Implemnation need to be done in the derived class for now
   if (task == "trim") {
     trim = value.toInt();
   }
   if (task == "speed") {
     speed = value.toInt();
   }
   left->setSpeed(speed);
   right->setSpeed(speed+trim);
   left->run(BACKWARD);
   right->run(BACKWARD);
   if (task = "stop") {
     left->setSpeed(0);
     right->setSpeed(0);
   }
}

void      readSerial() {
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
          handleSerialInstruction(task, value);
          apiBuffer = "";
        }
    }
  }
}
