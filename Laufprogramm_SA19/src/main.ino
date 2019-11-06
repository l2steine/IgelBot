#include <Arduino.h>
#include <PID_v1.h>

//Pinbelegung
#define PIN_A1 5 //Input A-Signal Encoder 1
#define PIN_B1 6 //Input B-Signal Encoder 1
#define PIN_I1 9 //Input I-Signal Encoder 1
#define PWM_1 13 //PWM-Ausgang für Motor 1
#define DIR_1 21 //Richtungsangabe für Motor 1
/* #define PIN_A2 17
#define PIN_B2 16
#define PIN_I2 15
#define PWM_2 12
#define DIR_2 20
#define PIN_A3 1
#define PIN_B3 0
#define PIN_I3 22
#define PWM_3 11
#define DIR_3 14
#define PIN_A4 24
#define PIN_B4 19
#define PIN_I4 18
#define PWM_4 10
#define DIR_4 23 */ //momentan soll nur 1 Bein getestet werden

/* double VLRegIn;   //PID Input vorne links
double VLRegOut;  //PID Output vorne rechts
double VRRegIn;
double VRRegOut;
double HLRegIn;
double HLRegOut;
double HRRegIn;
double HRRegOut;
*/  //momentan soll nur 1 Bein getestet werden

/* int toleranz = 10;    //PID toleranz
int schritt = 45;      //PID Schrittgrösse

double Kp=0.8, Ki=1, Kd=0; //Regler Verstärkungsfaktoren Sollposition

double Vornesollwert = 180;    //Zielposition der Beine
double Hintensollwert = 0;

PID VLReg(&VLRegIn, &VLRegOut, &Vornesollwert, Kp, Ki, Kd, DIRECT);   //Einrichten der Positionsregler (Momentanwert, Regelwert, Sollwert)
PID VRReg(&VRRegIn, &VRRegOut, &Hintensollwert, Kp, Ki, Kd, DIRECT);
PID HLReg(&HLRegIn, &HLRegOut, &Hintensollwert, Kp, Ki, Kd, DIRECT);
PID HRReg(&HRRegIn, &HRRegOut, &Vornesollwert, Kp, Ki, Kd, DIRECT);

*/ //momentan soll nur 1 Bein getestet werden

int i1, i2, i3, i4; ////A-Signal der Encoder für Beine (VL,VR,HL,HR)

volatile int encoder1Pos = 0;                 //Encoder Values auf 0 Stellen
/*volatile int encoder2Pos = 0;
volatile int encoder3Pos = 0;
volatile int encoder4Pos = 0;*/ //momentan soll nur 1 Bein getestet werden
volatile int encoderPinA1Last = LOW;  //Nötige Voreinstellungen für das Encoder Programm
/*volatile int encoderPinA2Last = LOW;
volatile int encoderPinA3Last = LOW;
volatile int encoderPinA4Last = LOW;*/ //momentan soll nur 1 Bein getestet werden
volatile int n1 = LOW;
/*volatile int n2 = LOW;
volatile int n3 = LOW;
volatile int n4 = LOW;*/ //momentan soll nur 1 Bein getestet werden

int speedHome = 63; //Homegeschwindigkeit

void setup() {
  pinMode (PIN_A1, INPUT);          //Pins definieren
  pinMode (PIN_B1, INPUT);
  pinMode (PIN_I1, INPUT);
  pinMode (PWM_1, OUTPUT);
  pinMode (DIR_1, OUTPUT);
  /*pinMode (PIN_A2, INPUT);
  pinMode (PIN_B2, INPUT);
  pinMode (PIN_I2, INPUT);
  pinMode (PWM_2, OUTPUT);
  pinMode (DIR_2, OUTPUT);
  pinMode (PIN_A3, INPUT);
  pinMode (PIN_B3, INPUT);
  pinMode (PIN_I3, INPUT);
  pinMode (PWM_3, OUTPUT);
  pinMode (DIR_3, OUTPUT);
  pinMode (PIN_A4, INPUT);
  pinMode (PIN_B4, INPUT);
  pinMode (PIN_I4, INPUT);
  pinMode (PWM_4, OUTPUT);
  pinMode (DIR_4, OUTPUT); */ //momentan soll nur 1 Bein getestet werden

  /* VLReg.SetOutputLimits(0,255);   //Geschwindigkeitsbegrenzungen der Regler definieren, PWM-Range definieren
  VRReg.SetOutputLimits(0,255);
  HLReg.SetOutputLimits(0,255);
  HRReg.SetOutputLimits(0,255);*/ //momentan soll nur 1 Bein getestet werden

  //Motoren initialisieren
  analogWrite (PWM_1, LOW); //Geschwindigkeit Motor 1 auf Null setzen
  digitalWrite (DIR_1, HIGH); //Drehrichtung Motor 1 vorwärts

  Serial.begin (9600);
  while(!Serial) //warten bis Serialport verbindet
  {
    delay(1);
  }
  Serial.println("Ready");
  delay(2000);

  analogWrite (PWM_1, speedHome); //Homingsequenz
  Serial.println("Homing Vorne Links...");
  while (i1 == 0) //Position kalibrieren
  {
  i1 = digitalRead(PIN_I1); //Indeximpuls für Referenzierung
  }
  while(encoder1Pos<200) //Wert 200 muss angepasst werden
  {
    n1 = digitalRead(PIN_A1);
    if((encoderPinA1Last == LOW) && (n1 == HIGH))
    {
      if (digitalRead(PIN_B1) == LOW)
      {
        encoder1Pos--;
      }
      else
      {
        encoder1Pos++;
      }
    }
    encoderPinA1Last = n1;
  }
  analogWrite (PWM_1, LOW); //Startposition erreicht
  Serial.println("Homing beendet");
  delay(1000);

  /* VLReg.SetMode(AUTOMATIC);           //Regler Modus einstellen
  VRReg.SetMode(AUTOMATIC);
  HLReg.SetMode(AUTOMATIC);
  HRReg.SetMode(AUTOMATIC); */

  /* encoder1Pos = 0;                    //Encoderwerte auf 0 setzen
  encoder2Pos = 0;
  encoder3Pos = 0;
  encoder4Pos = 0;
  delay(100);

  attachInterrupt(digitalPinToInterrupt(PIN_A1), encoder1, CHANGE);     //ISR-Definierung für Encoderabtastung
  attachInterrupt(digitalPinToInterrupt(PIN_A2), encoder2, CHANGE);     //ISR-Definierung für Encoderabtastung
  attachInterrupt(digitalPinToInterrupt(PIN_A3), encoder3, CHANGE);     //ISR-Definierung für Encoderabtastung
  attachInterrupt(digitalPinToInterrupt(PIN_A4), encoder4, CHANGE);     //ISR-Definierung für Encoderabtastung

  analogWrite (PWM_1, 63);
  analogWrite (PWM_2, 63);
  analogWrite (PWM_3, 63);
  analogWrite (PWM_4, 63); */ //Motoren starten

  }   //Ende void setup

void loop()
{
  /* VLRegIn = encoder1Pos;          //Encoderpositionen als Reglereingänge definieren
  VRRegIn = encoder2Pos;
  HLRegIn = encoder3Pos;
  HRRegIn = encoder4Pos;

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

  if (encoder1Pos >= Vornesollwert-toleranz && encoder2Pos >= Hintensollwert-toleranz && encoder3Pos >= Hintensollwert-toleranz && encoder4Pos >= Vornesollwert-toleranz)
  {
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

  /* void encoder2()  {             //ISR Encoderprogrammablauf
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
  encoderPinA4Last = n4; */
}
