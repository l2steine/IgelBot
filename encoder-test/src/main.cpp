#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Encoder.h>

#define M1 14
#define PIN_A 15
#define PIN_B 16
#define PIN_I 17

//#define DEBUG

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Encoder TestEnc(PIN_A, PIN_B);


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }
  Serial.println("Ready");
  pinMode(M1, OUTPUT);

  AFMS.begin();
  myMotor->setSpeed(30);
  myMotor->run(FORWARD);
}

long positionTestEnc = 100;
long newPos;

void loop() {
  // put your main code here, to run repeatedly:
  long newPos;
newPos = TestEnc.read();
if (newPos != positionTestEnc){
  Serial.print("Position = ");
  Serial.println(newPos);
  positionTestEnc = newPos;
  }
}
