#include <Arduino.h>
#include <PID_v1.h>

void encoder1(); //Funktionsdeklaration
void encoder2();
void encoder3();
void encoder4();

//Pinbelegung
#define PIN_A1 5 //Input A-Signal Encoder VL
#define PIN_B1 6 //Input B-Signal Encoder VL
#define PIN_I1 9 //Input I-Signal Encoder VL
#define PWM_1 13 //PWM-Ausgang für Motor VL
#define DIR_1 21 //Richtungsangabe für Motor VL
#define PIN_A2 17 //Input A-Signal Encoder VR
#define PIN_B2 16 //Input B-Signal Encoder VR
#define PIN_I2 15 //Input I-Signal Encoder VR
#define PWM_2 12 //PWM-Ausgang für Motor VR
#define DIR_2 20 //Richtungsangabe für Motor VR
#define PIN_A3 1 //Input A-Signal Encoder HL
#define PIN_B3 0 //Input B-Signal Encoder HL
#define PIN_I3 22 //Input I-Signal Encoder HL
#define PWM_3 11 //PWM-Ausgang für Motor HL
#define DIR_3 14 //Richtungsangabe für Motor HL
#define PIN_A4 24 //Input A-Signal Encoder HR
#define PIN_B4 19 //Input B-Signal Encoder HR
#define PIN_I4 18 //Input I-Signal Encoder HR
#define PWM_4 10 //PWM-Ausgang Motor HR
#define DIR_4 23 //Richtungsangabe für Motor HR

double VLRegIn;   //PID Input vorne links
double VLRegOut;  //PID Output vorne rechts
double VRRegIn;
double VRRegOut;
double HLRegIn;
double HLRegOut;
double HRRegIn;
double HRRegOut;

int toleranz = 10;    //PID toleranz
int schritt = 5;      //PID Schrittgrösse

double Kp=0.9, Ki=1, Kd=0; //Reglerfaktoren Sollposition
/*double Kp2=1.8, Ki2=1, Kd2=0; //Reglerfaktoren Sollgeschwindigkeit */

double Vornesollwert = 180;    //Zielposition der Beine
double Hintensollwert = 0;

PID VLReg(&VLRegIn, &VLRegOut, &Vornesollwert, Kp, Ki, Kd, DIRECT);   //Einrichten der Positionsregler (Momentanwert, Regelwert, Sollwert)
PID VRReg(&VRRegIn, &VRRegOut, &Hintensollwert, Kp, Ki, Kd, DIRECT);
PID HLReg(&HLRegIn, &HLRegOut, &Hintensollwert, Kp, Ki, Kd, DIRECT);
PID HRReg(&HRRegIn, &HRRegOut, &Vornesollwert, Kp, Ki, Kd, DIRECT);

/*PID VLSpeedReg(&VListDrehzahl, &VLSpeedRegOut, &VLRegOut, Kp2, Ki2, Kd2, DIRECT);   //Einrichten der Geschwindigkeitsregler (Istwert, Stellwert, Sollwert)
PID VRSpeedReg(&VRistDrehzahl, &VRSpeedRegOut, &VRRegOut, Kp2, Ki2, Kd2, DIRECT);
PID HLSpeedReg(&HListDrehzahl, &HLSpeedRegOut, &HLRegOut, Kp2, Ki2, Kd2, DIRECT);
PID HRSpeedReg(&HRistDrehzahl, &HRSpeedRegOut, &HRRegOut, Kp2, Ki2, Kd2, DIRECT); */

int i1, i2, i3, i4; ////A-Signal der Encoder für Beine (VL,VR,HL,HR)

volatile int encoder1Pos = 0; //Encoder Values auf 0 Stellen
volatile int encoder2Pos = 0;
volatile int encoder3Pos = 0;
volatile int encoder4Pos = 0;
volatile int encoderPinA1Last = LOW;  //Nötige Voreinstellungen für das Encoder Programm
volatile int encoderPinA2Last = LOW;
volatile int encoderPinA3Last = LOW;
volatile int encoderPinA4Last = LOW;
volatile int n1 = LOW;
volatile int n2 = LOW;
volatile int n3 = LOW;
volatile int n4 = LOW;

int speedHome = 60; //Homegeschwindigkeit
int speedWalk = 50; //Basislaufgeschwindigkeit

void setup()
{
  pinMode (PIN_A1, INPUT);          //Pins definieren
  pinMode (PIN_B1, INPUT);
  pinMode (PIN_I1, INPUT);
  pinMode (PWM_1, OUTPUT);
  pinMode (DIR_1, OUTPUT);
  pinMode (PIN_A2, INPUT);
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
  pinMode (DIR_4, OUTPUT);

  VLReg.SetOutputLimits(0,60);   //Geschwindigkeitsbegrenzungen der Regler definieren, PWM-Range definieren
  VRReg.SetOutputLimits(0,60);
  HLReg.SetOutputLimits(0,60);
  HRReg.SetOutputLimits(0,60);

  //Motoren initialisieren
  analogWrite (PWM_1, LOW);   //Geschwindigkeit Motor VL auf Null setzen
  digitalWrite (DIR_1, HIGH);  //Drehrichtung Motor VL vorwärts
  analogWrite (PWM_2, LOW);   //Geschwindigkeit Motor VR auf Null setzen
  digitalWrite (DIR_2, LOW);  //Drehrichtung Motor VR vorwärts
  analogWrite (PWM_3, LOW);   //Geschwindigkeit Motor HL auf Null setzen
  digitalWrite (DIR_3, HIGH);  //Drehrichtung Motor HL vorwärts
  analogWrite (PWM_4, LOW);   //Geschwindigkeit Motor HR auf Null setzen
  digitalWrite (DIR_4, LOW);  //Drehrichtung Motor HR vorwärts

  Serial.begin (9600);
  while(!Serial) //warten bis Serialport verbindet
  {
    delay(1);
  }
  Serial.println("Ready");
  delay(2000);

  analogWrite (PWM_1, speedHome); //Homingsequenz
  Serial.println("Homing vorne links...");
  while (i1 == 0) //Position kalibrieren
  {
  i1 = digitalRead(PIN_I1); //Indeximpuls für Referenzierung
  }
  while(encoder1Pos<970) //Wert muss angepasst werden
  {
    n1 = digitalRead(PIN_A1);
    if((encoderPinA1Last == LOW) && (n1 == HIGH))
    {
      if (digitalRead(PIN_B1) == LOW)
      {
        encoder1Pos++;
      }
      else
      {
        encoder1Pos--;
      }
    }
    encoderPinA1Last = n1;
  }
  analogWrite (PWM_1, LOW); //Startposition erreicht

  analogWrite (PWM_2, speedHome); //Homingsequenz
  Serial.println("Homing vorne rechts...");
  while (i2 == 0) //Position kalibrieren
  {
  i2 = digitalRead(PIN_I2); //Indeximpuls für Referenzierung
  }
  while(encoder2Pos<1025) //Wert muss angepasst werden
  {
    n2 = digitalRead(PIN_A2);
    if((encoderPinA2Last == LOW) && (n2 == HIGH))
    {
      if (digitalRead(PIN_B2) == LOW)
      {
        encoder2Pos--;
      }
      else
      {
        encoder2Pos++;
      }
    }
    encoderPinA2Last = n2;
  }
  analogWrite (PWM_2, LOW); //Startposition erreicht

  analogWrite (PWM_3, speedHome); //Homingsequenz
  Serial.println("Homing hinten links...");
  while (i3 == 0) //Position kalibrieren
  {
  i3 = digitalRead(PIN_I3); //Indeximpuls für Referenzierung
  }
  while(encoder3Pos<750) //Wert muss angepasst werden
  {
    n3 = digitalRead(PIN_A3);
    if((encoderPinA3Last == LOW) && (n3 == HIGH))
    {
      if (digitalRead(PIN_B3) == LOW)
      {
        encoder3Pos++;
      }
      else
      {
        encoder3Pos--;
      }
    }
    encoderPinA3Last = n3;
  }
  analogWrite (PWM_3, LOW); //Startposition erreicht

  analogWrite (PWM_4, speedHome); //Homingsequenz
  Serial.println("Homing hinten rechts...");
  while (i4 == 0) //Position kalibrieren
  {
  i4 = digitalRead(PIN_I4); //Indeximpuls für Referenzierung
  }
  while(encoder4Pos<880) //Wert muss angepasst werden
  {
    n4 = digitalRead(PIN_A4);
    if((encoderPinA4Last == LOW) && (n4 == HIGH))
    {
      if (digitalRead(PIN_B4) == LOW)
      {
        encoder4Pos--;
      }
      else
      {
        encoder4Pos++;
      }
    }
    encoderPinA4Last = n4;
  }
  analogWrite (PWM_4, LOW); //Startposition erreicht

  Serial.println("Homing beendet");
  delay(1000);

  VLReg.SetMode(AUTOMATIC);           //Regler Modus einstellen
  VRReg.SetMode(AUTOMATIC);
  HLReg.SetMode(AUTOMATIC);
  HRReg.SetMode(AUTOMATIC);

  encoder1Pos = 0;                    //Encoderwerte auf 0 setzen
  encoder2Pos = 0;
  encoder3Pos = 0;
  encoder4Pos = 0;
  delay(100);

  attachInterrupt(digitalPinToInterrupt(PIN_A1), encoder1, CHANGE);     //ISR-Definierung für Encoderabtastung
  attachInterrupt(digitalPinToInterrupt(PIN_A2), encoder2, CHANGE);     //ISR-Definierung für Encoderabtastung
  attachInterrupt(digitalPinToInterrupt(PIN_A3), encoder3, CHANGE);     //ISR-Definierung für Encoderabtastung
  attachInterrupt(digitalPinToInterrupt(PIN_A4), encoder4, CHANGE);     //ISR-Definierung für Encoderabtastung

  /*analogWrite (PWM_1, speedWalk);
  analogWrite (PWM_2, speedWalk);
  analogWrite (PWM_3, speedWalk);
  analogWrite (PWM_4, speedWalk);*/

}   //Ende void setup

void loop()
{
  /*
  delay(10);
  VLRegIn = encoder1Pos;          //Encoderpositionen als Reglereingänge definieren
  VRRegIn = encoder2Pos;
  HLRegIn = encoder3Pos;
  HRRegIn = encoder4Pos;

  VLReg.Compute();                //Regelsystem updaten
  VRReg.Compute();
  HLReg.Compute();
  HRReg.Compute();

  /*Serial.print("Aktuelle Position Vorne Links = ");       //Position auf SerialMonitor mitverfolgen
  Serial.println(encoder1Pos);
  Serial.print("Sollposition Vorne Links = ");          //Momentaner Sollwert
  Serial.println(Vornesollwert);
  analogWrite(PWM_1, VLRegOut);

  /*Serial.print("Position Vorne Rechts = ");
  Serial.println(encoder2Pos);
  Serial.print("Sollposition Vorne Rechts = ");
  Serial.println(Hintensollwert);
  analogWrite(PWM_2, VRRegOut);*/

  /*Serial.print("Position Hinten Links = ");
  Serial.println(encoder3Pos);
  Serial.print("Sollposition Hinten Links = ");
  Serial.println(Hintensollwert);
  analogWrite(PWM_3, HLRegOut);

  /*Serial.print("Position Hinten Rechts = ");
  Serial.println(encoder4Pos);
  Serial.print("Sollposition Hinten Rechts = ");
  Serial.println(Vornesollwert);
  analogWrite(PWM_4, HRRegOut);

  if (encoder1Pos >= Vornesollwert-toleranz && encoder2Pos >= Hintensollwert-toleranz && encoder3Pos >= Hintensollwert-toleranz && encoder4Pos >= Vornesollwert-toleranz)
  {
    Vornesollwert = Vornesollwert+schritt;
    Hintensollwert = Hintensollwert+schritt;
  }                                             //Beine bewegen wenn alle auf Position sind


  Vornesollwert+=schritt;
  Hintensollwert+=schritt; */
}

  void encoder1() //ISR Encoderprogrammablauf
  {
    n1 = digitalRead(PIN_A1);
    if((encoderPinA1Last == LOW) && (n1 == HIGH))
    {
      if (digitalRead(PIN_B1) == LOW)
      {
        encoder1Pos++;
      }
      else
      {
        encoder1Pos--;
      }
    }
    encoderPinA1Last = n1;
  }

  void encoder2() //ISR Encoderprogrammablauf
  {
    n2 = digitalRead(PIN_A2);
    if((encoderPinA2Last == LOW) && (n2 == HIGH))
    {
      if (digitalRead(PIN_B2) == LOW)
      {
        encoder2Pos--;
      }
      else
      {
        encoder2Pos++;
      }
    }
    encoderPinA2Last = n2;
  }

  void encoder3() //ISR Encoderprogrammablauf
  {
    n3 = digitalRead(PIN_A3);
    if((encoderPinA3Last == LOW) && (n3 == HIGH))
    {
      if (digitalRead(PIN_B3) == LOW)
      {
        encoder3Pos++;
      }
      else
      {
        encoder3Pos--;
      }
    }
    encoderPinA3Last = n3;
  }

  void encoder4() //ISR Encoderprogrammablauf
  {
    n4 = digitalRead(PIN_A4);
    if((encoderPinA4Last == LOW) && (n4 == HIGH))
    {
      if (digitalRead(PIN_B4) == LOW)
      {
        encoder4Pos--;
      }
      else
      {
        encoder4Pos++;
      }
    }
    encoderPinA4Last = n4;
  }
