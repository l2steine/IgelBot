#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <PID_v1.h>

#define M0 14              //Pinbelegung
#define PIN_A1 12
#define PIN_B1 11
#define PIN_I1 10
#define PIN_A2 9
#define PIN_B2 6
#define PIN_I2 5
#define PIN_A3 17
#define PIN_B3 16
#define PIN_I3 15
#define PIN_A4 24
#define PIN_B4 19
#define PIN_I4 18

double VLsollwert;
double VRsollwert;
double HLsollwert;
double HRsollwert;

double VLRegIn;   //Führende Position PID Input
double VLRegOut;  //Führende Position PID Output
double VRRegIn;
double VRRegOut;
double HLRegIn;
double HLRegOut;
double HRRegIn;
double HRRegOut;



double Kp=1, Ki=0.2, Kd=0.1;     //Regler Verstärkungsfaktoren

PID VLReg(&VLRegIn, &VLRegOut, &VLsollwert, Kp, Ki, Kd, DIRECT);
PID VRReg(&VRRegIn, &VRRegOut, &VRsollwert, Kp, Ki, Kd, DIRECT);
PID HLReg(&HLRegIn, &HLRegOut, &HLsollwert, Kp, Ki, Kd, DIRECT);
PID HRReg(&HRRegIn, &HRRegOut, &HRsollwert, Kp, Ki, Kd, DIRECT);

Adafruit_MotorShield AFMS = Adafruit_MotorShield();     //Motor einrichten
Adafruit_DCMotor *BeinVL = AFMS.getMotor(1);
Adafruit_DCMotor *BeinVR = AFMS.getMotor(2);
Adafruit_DCMotor *BeinHL = AFMS.getMotor(3);
Adafruit_DCMotor *BeinHR = AFMS.getMotor(4);

int i1, i2, i3, i4;

volatile int encoder1Pos = 0;                 //Encoder Values
volatile int encoder2Pos = 0;
volatile int encoder3Pos = 0;
volatile int encoder4Pos = 0;
volatile int encoderPinA1Last = LOW;
volatile int encoderPinA2Last = LOW;
volatile int encoderPinA3Last = LOW;
volatile int encoderPinA4Last = LOW;
volatile int n1 = LOW;
volatile int n2 = LOW;
volatile int n3 = LOW;
volatile int n4 = LOW;

void setup() {
  pinMode (PIN_A1, INPUT);
  pinMode (PIN_B1, INPUT);
  pinMode (PIN_I1, INPUT);
  pinMode (PIN_A2, INPUT);
  pinMode (PIN_B2, INPUT);
  pinMode (PIN_I2, INPUT);
  pinMode (PIN_A3, INPUT);
  pinMode (PIN_B3, INPUT);
  pinMode (PIN_I3, INPUT);
  pinMode (PIN_A4, INPUT);
  pinMode (PIN_B4, INPUT);
  pinMode (PIN_I4, INPUT);

  VLReg.SetOutputLimits(0,55);
  VRReg.SetOutputLimits(0,55);
  HLReg.SetOutputLimits(0,55);
  HRReg.SetOutputLimits(0,55);

  Serial.begin (9600);
    while(!Serial){
      delay(1);
    }
Serial.println("Ready");
pinMode(M0,OUTPUT);       //Motor konfigurieren
AFMS.begin();
BeinVL->setSpeed(40);
BeinVR->setSpeed(40);
BeinHL->setSpeed(40);
BeinHR->setSpeed(40);

BeinVL->run(FORWARD);
while (i1 == 0) {        //Position Kalibrieren
  Serial.println("Homing Vorne Links...");
  i1 = digitalRead(PIN_I1);
    }
while(encoder1Pos<195)  {
  Serial.println("Homing Vorne Links Phase 2...");
  n1 = digitalRead(PIN_A1);
    if((encoderPinA1Last == LOW) && (n1 == HIGH)) {
    if (digitalRead(PIN_B1) == LOW) {
      encoder1Pos++;
    } else {
      encoder1Pos--; }
  }
  encoderPinA1Last = n1;
}
BeinVL ->run(RELEASE);           //Startposition erreicht

BeinVR ->run(FORWARD);
while (i2 == 0) {        //Position Kalibrieren
  Serial.println("Homing Vorne Rechts...");
  i2 = digitalRead(PIN_I2);
        }
/*while(encoder2Pos<210)  {     //Serial.println("Rechts auf Startposition...");
  Serial.println("Homing Vorne Rechts Phase 2...");
  n2 = digitalRead(PIN_A2);
  if((encoderPinA2Last == LOW) && (n2 == HIGH)) {
  if (digitalRead(PIN_B2) == LOW) {
    encoder2Pos--;
  } else {
    encoder2Pos++; }
  }
  encoderPinA2Last = n2;
}*/
BeinVR->run(RELEASE);          //Startposition erreicht

/*BeinHL ->run(FORWARD);
while (i3 == 0) {        //Position Kalibrieren
  Serial.println("Homing Hinten Links...");
  i3 = digitalRead(PIN_I3);
        }
while(encoder3Pos<225)  {     //Serial.println("Rechts auf Startposition...");
  Serial.println("Homing Hinten Links Phase 2...");
  n3 = digitalRead(PIN_A3);
  if((encoderPinA3Last == LOW) && (n3 == HIGH)) {
  if (digitalRead(PIN_B3) == LOW) {
    encoder3Pos++;
  } else {
    encoder3Pos--; }
  }
  encoderPinA3Last = n3;
}
BeinHL->run(RELEASE);*/

BeinHR ->run(FORWARD);
while (i4 == 0) {        //Position Kalibrieren
  Serial.println("Homing Hinten Rechts...");
  i4 = digitalRead(PIN_I4);
        }
while(encoder4Pos<195)  {     //Serial.println("Rechts auf Startposition...");
  Serial.println("Homing Hinten Rechts Phase 2...");
  n4 = digitalRead(PIN_A4);
  if((encoderPinA4Last == LOW) && (n4 == HIGH)) {
  if (digitalRead(PIN_B4) == LOW) {
    encoder4Pos--;
  } else {
    encoder4Pos++; }
  }
  encoderPinA4Last = n4;
}
BeinHR->run(RELEASE);

Serial.println("Homing beendet");
delay(500);

VLReg.SetMode(AUTOMATIC);
VRReg.SetMode(AUTOMATIC);
HLReg.SetMode(AUTOMATIC);
HRReg.SetMode(AUTOMATIC);

encoder1Pos = 0;
encoder2Pos = 0;
encoder3Pos = 0;
encoder4Pos = 0;
delay(250);

attachInterrupt(digitalPinToInterrupt(PIN_A1), encoder1, CHANGE);     //ISR Encoderabtast
attachInterrupt(digitalPinToInterrupt(PIN_A2), encoder2, CHANGE);     //ISR Encoderabtast
attachInterrupt(digitalPinToInterrupt(PIN_A3), encoder3, CHANGE);     //ISR Encoderabtast
attachInterrupt(digitalPinToInterrupt(PIN_A4), encoder4, CHANGE);     //ISR Encoderabtast

BeinVL ->run(FORWARD);
BeinVR ->run(FORWARD);
BeinHL ->run(FORWARD);
BeinHR ->run(FORWARD);

double walkarray[4] = {0};
}

void loop() {
VLRegIn = encoder1Pos;
VRRegIn = encoder2Pos;
HLRegIn = encoder3Pos;
HRRegIn = encoder4Pos;

VLReg.Compute();
VRReg.Compute();
HLReg.Compute();
HRReg.Compute();

delay(100);
Serial.print("Position Vorne Links = ");       //Position auf SerialMonitor mitverfolgen
Serial.println(encoder1Pos);
BeinVL ->setSpeed(VLRegOut);

Serial.print("Position Vorne Rechts = ");
Serial.println(encoder2Pos);
BeinVR ->setSpeed(VRRegOut);

Serial.print("Position Hinten Links = ");
Serial.println(encoder3Pos);
BeinHL ->setSpeed(HLRegOut);

Serial.print("Position Hinten Rechts = ");
Serial.println(encoder4Pos);
BeinHR ->setSpeed(HRRegOut);
}

void encoder1()  {             //ISR Encoderprogrammablauf
  n1 = digitalRead(PIN_A1);
    if((encoderPinA1Last == LOW) && (n1 == HIGH)) {
    if (digitalRead(PIN_B1) == LOW) {
      encoder1Pos++;
    } else {
      encoder1Pos--; }
  i1 = digitalRead(PIN_I1);
  }
  encoderPinA1Last = n1;
}

void encoder2()  {             //ISR Encoderprogrammablauf
  n2 = digitalRead(PIN_A2);
    if((encoderPinA2Last == LOW) && (n2 == HIGH)) {
    if (digitalRead(PIN_B2) == LOW) {
      encoder2Pos--;
    } else {
      encoder2Pos++; }
  i2 = digitalRead(PIN_I2);
  }
  encoderPinA2Last = n2;
}

void encoder3()  {             //ISR Encoderprogrammablauf
  n3 = digitalRead(PIN_A3);
    if((encoderPinA3Last == LOW) && (n3 == HIGH)) {
    if (digitalRead(PIN_B3) == LOW) {
      encoder3Pos++;
    } else {
      encoder3Pos--; }
  i3 = digitalRead(PIN_I3);
  }
  encoderPinA3Last = n3;
}

void encoder4()  {             //ISR Encoderprogrammablauf
  n4 = digitalRead(PIN_A4);
    if((encoderPinA4Last == LOW) && (n4 == HIGH)) {
    if (digitalRead(PIN_B4) == LOW) {
      encoder4Pos--;
    } else {
      encoder4Pos++; }
  i4 = digitalRead(PIN_I4);
  }
  encoderPinA4Last = n4;
}
