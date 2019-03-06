#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define M1 14
#define PIN_A 15
#define PIN_B 16
#define PIN_I 17

int val = 0;
int encPos = 0;
int ipeaks = 0;
int encPinALast = LOW;
int encPinILast = LOW;
int a = LOW;
int i = LOW;
int turns = 0;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }
  Serial.println("Ready");
  pinMode(M1, OUTPUT);
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  pinMode(PIN_I, INPUT);
  AFMS.begin();
  myMotor->setSpeed(40);
  myMotor->run(FORWARD);
}


void loop() {
  // put your main code here, to run repeatedly:
a = digitalRead(PIN_A);
  i = digitalRead(PIN_I);
  if ((encPinILast == LOW) &&( i == HIGH)) {
    Serial.println(encPos);
    encPos = 0;
    turns++;
    //Serial.print (turns);
    //Serial.print (" / ");
  }
  if ((encPinALast == LOW) && (a == HIGH)) {
    if (digitalRead(PIN_B) == LOW) {
      encPos++;
    } else {
      encPos--;
    }
  }
