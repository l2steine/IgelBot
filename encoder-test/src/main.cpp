#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define M0 14
#define PIN_EN1A 15
#define PIN_EN1B 16
#define PIN_EN1I 17
#define PIN_EN2A 18
#define PIN_EN2B 19
#define PIN_EN2I 24
#define PIN_EN3A 12
#define PIN_EN3B 11
#define PIN_EN3I 10
#define PIN_EN4A 9
#define PIN_EN4B 6
#define PIN_EN4I 5

//#define DEBUG
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *hintenL = AFMS.getMotor(1);
Adafruit_DCMotor *vorneL = AFMS.getMotor(2);
Adafruit_DCMotor *hintenR = AFMS.getMotor(3);
Adafruit_DCMotor *vorneR = AFMS.getMotor(4);
Encoder EncVL(PIN_EN1A, PIN_EN1B);
Encoder EncVR(PIN_EN2A, PIN_EN2B);
Encoder EncHL(PIN_EN3A, PIN_EN3B);
Encoder EncHR(PIN_EN4A, PIN_EN4B);

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }
  Serial.println("Ready");
  pinMode(M0, OUTPUT);

  AFMS.begin();
  vorneL->setSpeed(50);
  vorneL->run(FORWARD);
  /*hintenL->setSpeed(50);
  hintenL->run(FORWARD);*/
  vorneR->setSpeed(50);
  vorneR->run(FORWARD);
  hintenR->setSpeed(50);
  hintenR->run(FORWARD);
}


long positionEncVL = 0;
long positionEncVR = 0;
long positionEncHL = 0;
long positionEncHR = 0;


void loop() {
  // put your main code here, to run repeatedly:
long newPosVL, newPosVR, newPosHL, newPosHR;

newPosVL = EncVL.read();
newPosVR = EncVR.read();
newPosHL = EncHL.read();
newPosHR = EncHR.read();

if (newPosVL != positionEncVL || newPosVR != positionEncVR || newPosHL != positionEncHL || newPosHR != positionEncHL){
  delay(500);
  Serial.print("Position VorneL = ");
  Serial.println(newPosVL);

  delay(500);
  Serial.print("Position VorneR = ");
  Serial.println((-1)*newPosVR);

  delay(500);
  Serial.print("Position HintenL = ");
  Serial.println(newPosHL);

  delay(500);
  Serial.print("Position HintenR = ");
  Serial.println((-1)*newPosHR);

  positionEncVL = newPosVL;
  positionEncVR = newPosVR;
  positionEncHL = newPosHL;
  positionEncHR = newPosHR;
  }

}
