#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <PID_v1.h>

#define M0 14              //Pinbelegung
#define PIN_A1 10
#define PIN_B1 11
#define PIN_I1 12
#define PIN_A2 17
#define PIN_B2 16
#define PIN_I2 15
#define PIN_A3 9
#define PIN_B3 6
#define PIN_I3 5
#define PIN_A4 24
#define PIN_B4 19
#define PIN_I4 18

double VLRegIn;   //PID Input
double VLRegOut;  //PID Output
double VRRegIn;
double VRRegOut;
double HLRegIn;
double HLRegOut;
double HRRegIn;
double HRRegOut;

int toleranz = 10;    //PID toleranz
int schritt = 45;      //PID Schrittgrösse

double Kp=0.8, Ki=1, Kd=0;     //Regler Verstärkungsfaktoren

double Vornesollwert = 180;    //Zielpositione der Beine
double Hintensollwert = 0;


PID VLReg(&VLRegIn, &VLRegOut, &Vornesollwert, Kp, Ki, Kd, DIRECT);       //Einrichten der Regler
PID VRReg(&VRRegIn, &VRRegOut, &Hintensollwert, Kp, Ki, Kd, DIRECT);
PID HLReg(&HLRegIn, &HLRegOut, &Hintensollwert, Kp, Ki, Kd, DIRECT);
PID HRReg(&HRRegIn, &HRRegOut, &Vornesollwert, Kp, Ki, Kd, DIRECT);

Adafruit_MotorShield AFMS = Adafruit_MotorShield();     //Motor eine Nummer am Motor Shield zuordnen
Adafruit_DCMotor *BeinVL = AFMS.getMotor(1);
Adafruit_DCMotor *BeinVR = AFMS.getMotor(2);
Adafruit_DCMotor *BeinHL = AFMS.getMotor(4);
Adafruit_DCMotor *BeinHR = AFMS.getMotor(3);

int i1, i2, i3, i4;

volatile int encoder1Pos = 0;                 //Encoder Values auf 0 Stellen
volatile int encoder2Pos = 0;
volatile int encoder3Pos = 0;
volatile int encoder4Pos = 0;
volatile int encoderPinA1Last = LOW;          //Nötige Voreinstellungen für das Encoder Programm
volatile int encoderPinA2Last = LOW;
volatile int encoderPinA3Last = LOW;
volatile int encoderPinA4Last = LOW;
volatile int n1 = LOW;
volatile int n2 = LOW;
volatile int n3 = LOW;
volatile int n4 = LOW;


void setup() {
  pinMode (PIN_A1, INPUT);          //Pins definieren
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

  VLReg.SetOutputLimits(0,140);           //Geschwindigkeitsbegrenzungen der Regler definieren
  VRReg.SetOutputLimits(0,140);
  HLReg.SetOutputLimits(0,140);
  HRReg.SetOutputLimits(0,140);

  Serial.begin (9600);
    while(!Serial){
      delay(1);
    }
Serial.println("Ready");
pinMode(M0,OUTPUT);       //Motor konfigurieren
AFMS.begin();
BeinVL->setSpeed(150);        //Geschwindigkeit beim Homen
BeinVR->setSpeed(150);
BeinHL->setSpeed(150);
BeinHR->setSpeed(150);

BeinVL->run(FORWARD);         //Homing Sequenz
Serial.println("Homing Vorne Links...");
while (i1 == 0) {        //Position Kalibrieren
  i1 = digitalRead(PIN_I1);
    }
while(encoder1Pos<200)  {
  n1 = digitalRead(PIN_A1);
    if((encoderPinA1Last == LOW) && (n1 == HIGH)) {
    if (digitalRead(PIN_B1) == LOW) {
      encoder1Pos--;
    } else {
      encoder1Pos++; }
  }
  encoderPinA1Last = n1;
}
BeinVL ->run(RELEASE);           //Startposition erreicht

BeinVR ->run(FORWARD);
Serial.println("Homing Vorne Rechts...");
while (i2 == 0) {        //Position Kalibrieren
  i2 = digitalRead(PIN_I2);
        }
while(encoder2Pos<150)  {
  n2 = digitalRead(PIN_A2);
  if((encoderPinA2Last == LOW) && (n2 == HIGH)) {
  if (digitalRead(PIN_B2) == LOW) {
    encoder2Pos++;
  } else {
    encoder2Pos--; }
  }
  encoderPinA2Last = n2;
}
BeinVR->run(RELEASE);          //Startposition erreicht

BeinHL ->run(FORWARD);
Serial.println("Homing Hinten Links...");
while (i3 == 0) {        //Position Kalibrieren
  i3 = digitalRead(PIN_I3);
        }
while(encoder3Pos<10)  {     //Serial.println("Rechts auf Startposition...");
  n3 = digitalRead(PIN_A3);
  if((encoderPinA3Last == LOW) && (n3 == HIGH)) {
  if (digitalRead(PIN_B3) == LOW) {
    encoder3Pos--;
  } else {
    encoder3Pos++; }
  }
  encoderPinA3Last = n3;
}
BeinHL->run(RELEASE);

BeinHR ->run(FORWARD);
Serial.println("Homing Hinten Rechts...");
while (i4 == 0) {        //Position Kalibrieren
  i4 = digitalRead(PIN_I4);
        }
while(encoder4Pos<155)  {     //Serial.println("Rechts auf Startposition...");
  n4 = digitalRead(PIN_A4);
  if((encoderPinA4Last == LOW) && (n4 == HIGH)) {
  if (digitalRead(PIN_B4) == LOW) {
    encoder4Pos++;
  } else {
    encoder4Pos--; }
  }
  encoderPinA4Last = n4;
}

BeinHR->run(RELEASE);

Serial.println("Homing beendet");
delay(100);

VLReg.SetMode(AUTOMATIC);           //Regler Modus einstellen
VRReg.SetMode(AUTOMATIC);
HLReg.SetMode(AUTOMATIC);
HRReg.SetMode(AUTOMATIC);

encoder1Pos = 0;                    //Encoderwerte auf 0 setzen
encoder2Pos = 0;
encoder3Pos = 0;
encoder4Pos = 0;
delay(500);

attachInterrupt(digitalPinToInterrupt(PIN_A1), encoder1, CHANGE);     //ISR Encoderabtast
attachInterrupt(digitalPinToInterrupt(PIN_A2), encoder2, CHANGE);     //ISR Encoderabtast
attachInterrupt(digitalPinToInterrupt(PIN_A3), encoder3, CHANGE);     //ISR Encoderabtast
attachInterrupt(digitalPinToInterrupt(PIN_A4), encoder4, CHANGE);     //ISR Encoderabtast

BeinVL ->run(FORWARD);          //Motoren starten
BeinVR ->run(FORWARD);
BeinHL ->run(FORWARD);
BeinHR ->run(FORWARD);
}

void loop() {
VLRegIn = encoder1Pos;          //Regler Encoderpositionen als Reglereingänge definieren
VRRegIn = encoder2Pos;
HLRegIn = encoder3Pos;
HRRegIn = encoder4Pos;

/*if(LeadVornesollwert<encoder1Pos) {     //Richtung ändern falls nötig
  BeinVL ->run(BACKWARD);
  } else {
  BeinVL ->run(FORWARD);
}

if(LeadHintensollwert<encoder2Pos) {
  BeinVR ->run(BACKWARD);
  } else {
  BeinVR ->run(FORWARD);
}

if(FollowVornesollwert<encoder4Pos) {
  BeinHR ->run(BACKWARD);
  } else {
  BeinHR ->run(FORWARD);
}

if(FollowHintensollwert<encoder3Pos) {
  BeinHL ->run(BACKWARD);
  } else {
  BeinHL ->run(FORWARD);
}
*/

VLReg.Compute();                //Regelsystem updaten
VRReg.Compute();
HLReg.Compute();
HRReg.Compute();

delay(100);
Serial.print("Aktuelle Position Vorne Links = ");       //Position auf SerialMonitor mitverfolgen
Serial.println(encoder1Pos);
Serial.print("Sollposition Vorne Links = ");          //Momentaner Sollwert
Serial.println(Vornesollwert);
Serial.print("Anzahl Umdrehungen Vorne Links = ");    //Anzahl Umdrehungen auf SerialMonitor mitverfolgen
Serial.println(encoder1Pos/360);
Serial.println(" ");
BeinVL ->setSpeed(VLRegOut);

Serial.print("Position Vorne Rechts = ");
Serial.println(encoder2Pos);
Serial.print("Sollposition Vorne Rechts = ");
Serial.println(Hintensollwert);
Serial.print("Anzahl Umdrehungen Vorne Rechts = ");
Serial.println(encoder2Pos/360);
Serial.println(" ");
BeinVR ->setSpeed(VRRegOut);

Serial.print("Position Hinten Links = ");
Serial.println(encoder3Pos);
Serial.print("Sollposition Hinten Links = ");
Serial.println(Hintensollwert);
Serial.print("Anzahl Umdrehungen Hinten Links = ");
Serial.println(encoder3Pos/360);
Serial.println(" ");
BeinHL ->setSpeed(HLRegOut);

Serial.print("Position Hinten Rechts = ");
Serial.println(encoder4Pos);
Serial.print("Sollposition Hinten Rechts = ");
Serial.println(Vornesollwert);
Serial.print("Anzahl Umdrehungen Hinten Rechts = ");
Serial.println(encoder4Pos/360);
Serial.println(" ");
BeinHR ->setSpeed(HRRegOut);

if (encoder1Pos >= Vornesollwert-toleranz && encoder2Pos >= Hintensollwert-toleranz && encoder3Pos >= Hintensollwert-toleranz && encoder4Pos >= Vornesollwert-toleranz){
  Vornesollwert = Vornesollwert+schritt;
  Hintensollwert = Hintensollwert+schritt;
}                                             //Beine bewegen wenn alle auf Position sind

}

void encoder1()  {             //ISR Encoderprogrammablauf
  n1 = digitalRead(PIN_A1);
    if((encoderPinA1Last == LOW) && (n1 == HIGH)) {
    if (digitalRead(PIN_B1) == LOW) {
      encoder1Pos--;
    } else {
      encoder1Pos++; }
  }
  encoderPinA1Last = n1;
}

void encoder2()  {             //ISR Encoderprogrammablauf
  n2 = digitalRead(PIN_A2);
    if((encoderPinA2Last == LOW) && (n2 == HIGH)) {
    if (digitalRead(PIN_B2) == LOW) {
      encoder2Pos++;
    } else {
      encoder2Pos--; }
  }
  encoderPinA2Last = n2;
}

void encoder3()  {             //ISR Encoderprogrammablauf
  n3 = digitalRead(PIN_A3);
    if((encoderPinA3Last == LOW) && (n3 == HIGH)) {
    if (digitalRead(PIN_B3) == LOW) {
      encoder3Pos--;
    } else {
      encoder3Pos++; }
  }
  encoderPinA3Last = n3;
}

void encoder4()  {             //ISR Encoderprogrammablauf
  n4 = digitalRead(PIN_A4);
    if((encoderPinA4Last == LOW) && (n4 == HIGH)) {
    if (digitalRead(PIN_B4) == LOW) {
      encoder4Pos++;
    } else {
      encoder4Pos--; }
  }
  encoderPinA4Last = n4;
}
