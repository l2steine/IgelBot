#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define M0 14
#define PIN_A 17
#define PIN_B 16
#define PIN_I 15

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Bein = AFMS.getMotor(1);

int val;
int a;
int b;
int i;

volatile int encoderPos = 0;
int encoderPinALast = LOW;
int n = LOW;

void setup() {
  pinMode (PIN_A, INPUT);
  pinMode (PIN_B, INPUT);
  Serial.begin (9600);
    while(!Serial){
      delay(1);
    }
Serial.println("Ready");
pinMode(M0,OUTPUT);
AFMS.begin();
Bein->setSpeed(35);
Bein->run(FORWARD);
while (i == 0) {
  delay(1);
  Serial.println("Homing...");
  i = digitalRead(PIN_I);
    }
Bein ->run(RELEASE);
delay(1000);
Bein->run(FORWARD);
delay(500);
encoderPos = 0;
}

void loop() {
  n = digitalRead(PIN_A);
  if((encoderPinALast == LOW) && (n == HIGH)) {
    if (digitalRead(PIN_B) == LOW) {
      encoderPos--;
    } else {
      encoderPos++; }
  i = digitalRead(PIN_I);
  Serial.print("Bein Position = ");
  Serial.println((-1)*encoderPos);
  }
  encoderPinALast = n;
}
