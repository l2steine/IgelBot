#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define M0 14
#define PIN_EN1A 12
#define PIN_EN1B 11
#define PIN_EN1I 10
#define PIN_EN2A 9
#define PIN_EN2B 6
#define PIN_EN2I 5
#define PIN_EN3A 17
#define PIN_EN3B 16
#define PIN_EN3I 15
#define PIN_EN4A 24
#define PIN_EN4B 19
#define PIN_EN4I 18

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *hintenL = AFMS.getMotor(1);
Adafruit_DCMotor *vorneL = AFMS.getMotor(2);
Adafruit_DCMotor *hintenR = AFMS.getMotor(3);
Adafruit_DCMotor *vorneR = AFMS.getMotor(4);

int val;
int encoder1Pos = 0;
int encoder1PinALast = LOW;
int encoder2Pos = 0;
int encoder2PinALast = LOW;
int encoder3Pos = 0;
int encoder3PinALast = LOW;
int encoder4Pos = 0;
int encoder4PinALast = LOW;
int n1 = LOW;
int n2 = LOW;
int n3 = LOW;
int n4 = LOW;

void setup() {
  pinMode (PIN_EN1A, INPUT);
  pinMode (PIN_EN1B, INPUT);
  Serial.begin (9600);
    while(!Serial){
      delay(1);
    }
Serial.println("Ready");
pinMode(M0,OUTPUT);
AFMS.begin();
vorneL->setSpeed(35);
vorneL->run(FORWARD);
hintenL->setSpeed(35);
hintenL->run(FORWARD);
vorneR->setSpeed(35);
vorneR->run(FORWARD);
hintenR->setSpeed(35);
hintenR->run(FORWARD);
}

void loop() {
  n1 = digitalRead(PIN_EN1A);
  if ((encoder1PinALast == LOW) && (n1 == HIGH)) {
    if (digitalRead(PIN_EN1B) == LOW) {
      encoder1Pos--;
    } else {
      encoder1Pos++;
    }
    delay(500);
    Serial.print("Anzahl Umdrehungen VL = ");
    Serial.println(encoder1Pos);
  }
  encoder1PinALast = n1;

  n2 = digitalRead(PIN_EN2A);
  if ((encoder2PinALast == LOW) && (n2 == HIGH)) {
    if (digitalRead(PIN_EN2B) == LOW) {
      encoder2Pos--;
    } else {
      encoder2Pos++;
    }
    delay(500);
    Serial.print("Anzahl Umdrehungen VR = ");
    Serial.println((-1)*encoder2Pos);
  }
  encoder2PinALast = n2;

  n3 = digitalRead(PIN_EN3A);
  if ((encoder3PinALast == LOW) && (n3 == HIGH)) {
    if (digitalRead(PIN_EN3B) == LOW) {
      encoder3Pos--;
    } else {
      encoder3Pos++;
    }
    delay(500);
    Serial.print("Anzahl Umdrehungen VL = ");
    Serial.println((-1)*encoder3Pos);
  }
  encoder2PinALast = n3;

  n4 = digitalRead(PIN_EN4A);
  if ((encoder4PinALast == LOW) && (n4 == HIGH)) {
    if (digitalRead(PIN_EN4B) == LOW) {
      encoder4Pos--;
    } else {
      encoder4Pos++;
    }
    delay(500);
    Serial.print("Anzahl Umdrehungen HR = ");
    Serial.println((-1)*encoder4Pos);
  }
  encoder2PinALast = n4;
}
