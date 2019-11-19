#include <Arduino.h>
#include <PID_v1.h>

//Encoderfunktionen deklarieren; lesen Encoderveränderungen aus
/*void encoderLinks(volatile long encoderArr[][5], volatile int i);
void encoderRechts(volatile long encoderArr[][5], volatile int i);
void encoder1L(volatile long encoderArr[][5], volatile int i);*/
void encoder1L();
void encoder2R();
void encoder3L();
void encoder4R();

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

//Deklaration der Reglerin- und Outputs (muessen double sein!)
double VLRegIn;
double VLRegOut;
double VRRegIn;
double VRRegOut;
double HLRegIn;
double HLRegOut;
double HRRegIn;
double HRRegOut;

//Regelparameter
double Kp=1.5, Ki=0, Kd=0;

//Zielposition der Beine
double Vornesollwert = 180;
double Hintensollwert = 0;

//PID-Positionsregler definieren (Momentanwert, Regelwert, Sollwert)
PID VLReg(&VLRegIn, &VLRegOut, &Vornesollwert, Kp, Ki, Kd, DIRECT);
PID VRReg(&VRRegIn, &VRRegOut, &Hintensollwert, Kp, Ki, Kd, DIRECT);
PID HLReg(&HLRegIn, &HLRegOut, &Hintensollwert, Kp, Ki, Kd, DIRECT);
PID HRReg(&HRRegIn, &HRRegOut, &Vornesollwert, Kp, Ki, Kd, DIRECT);

int toleranz = 10; //PID Toleranz
int schritt = 5; //PID Schrittgrösse

int i1, i2, i3, i4; //I-Signal der Encoder für Beine (VL,VR,HL,HR)

volatile long encoder1Pos = 0; //Encoder Values auf 0 Stellen
volatile long encoder2Pos = 0;
volatile long encoder3Pos = 0;
volatile long encoder4Pos = 0;
volatile long encoderPinA1Last = LOW;  //Nötige Voreinstellungen für das Encoder Programm
volatile long encoderPinA2Last = LOW;
volatile long encoderPinA3Last = LOW;
volatile long encoderPinA4Last = LOW;
volatile int n1 = LOW;
volatile int n2 = LOW;
volatile int n3 = LOW;
volatile int n4 = LOW;
volatile int i; //Laufvariable für Encoderfunktionerzeugung

volatile long encoderArr[4][5] = {{n1, PIN_A1, encoderPinA1Last, PIN_B1, encoder1Pos},
                                  {n2, PIN_A2, encoderPinA2Last, PIN_B2, encoder2Pos},
                                  {n3, PIN_A3, encoderPinA3Last, PIN_B3, encoder3Pos},
                                  {n4, PIN_A4, encoderPinA4Last, PIN_B4, encoder4Pos}};

int speedWalk = 80; //Basisgeschwindigkeit

void setup()
{
  //Pins definieren
  pinMode (PIN_A1, INPUT);
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

  //PWM-Outputbegrenzung des Reglers
  VLReg.SetOutputLimits(0,255);
  VRReg.SetOutputLimits(0,255);
  HLReg.SetOutputLimits(0,255);
  HRReg.SetOutputLimits(0,255);

  //Motoren initialisieren
  analogWrite (PWM_1, LOW); //Geschwindigkeit Motor VL auf Null setzen
  digitalWrite (DIR_1, HIGH); //Drehrichtung Motor VL vorwärts
  analogWrite (PWM_2, LOW); //Geschwindigkeit Motor VR auf Null setzen
  digitalWrite (DIR_2, LOW); //Drehrichtung Motor VR vorwärts
  analogWrite (PWM_3, LOW); //Geschwindigkeit Motor HL auf Null setzen
  digitalWrite (DIR_3, HIGH); //Drehrichtung Motor HL vorwärts
  analogWrite (PWM_4, LOW); //Geschwindigkeit Motor HR auf Null setzen
  digitalWrite (DIR_4, LOW); //Drehrichtung Motor HR vorwärts

  Serial.begin (9600);
  while(!Serial) //warten bis Serialport verbindet
  {
    delay(1);
  }
  Serial.println("Ready");
  delay(2000);

  //Homing vorne links
  analogWrite (PWM_1, speedWalk);
  Serial.println("Homing vorne links...");
  while (i1 == 0) //Position kalibrieren
  {
  i1 = digitalRead(PIN_I1); //Indeximpuls für Referenzierung
  }
  while(encoder1Pos<1130) //Wert muss angepasst werden
  {
    encoder1L();
    //encoder1L(encoderArr, i);
  }
  analogWrite (PWM_1, LOW); //Startposition erreicht

  //Homings vorne rechts
  analogWrite (PWM_2, speedWalk);
  Serial.println("Homing vorne rechts...");
  while (i2 == 0) //Position kalibrieren
  {
  i2 = digitalRead(PIN_I2); //Indeximpuls für Referenzierung
  }
  while(encoder2Pos<1080) //Wert muss angepasst werden
  {
    encoder2R();
  }
  analogWrite (PWM_2, LOW); //Startposition erreicht

  //Homing hinten links
  analogWrite (PWM_3, speedWalk);
  Serial.println("Homing hinten links...");
  while (i3 == 0) //Position kalibrieren
  {
  i3 = digitalRead(PIN_I3); //Indeximpuls für Referenzierung
  }
  while(encoder3Pos<980) //Wert muss angepasst werden
  {
    encoder3L();
  }
  analogWrite (PWM_3, LOW); //Startposition erreicht

  //Homing hinten rechts
  analogWrite (PWM_4, speedWalk);
  Serial.println("Homing hinten rechts...");
  while (i4 == 0) //Position kalibrieren
  {
  i4 = digitalRead(PIN_I4); //Indeximpuls für Referenzierung
  }
  while(encoder4Pos<1010) //Wert muss angepasst werden
  {
    encoder4R();
  }
  analogWrite (PWM_4, LOW); //Startposition erreicht

  Serial.println("Homing beendet");
  delay(1000);

  //Reglermodus einstellen
  VLReg.SetMode(AUTOMATIC);
  VRReg.SetMode(AUTOMATIC);
  HLReg.SetMode(AUTOMATIC);
  HRReg.SetMode(AUTOMATIC);

  //Encoderwerte auf Startposition nullen
  encoder1Pos = 0;
  encoder2Pos = 0;
  encoder3Pos = 0;
  encoder4Pos = 0;
  delay(100);

  //ISR für Encoderabfrage definieren
  attachInterrupt(digitalPinToInterrupt(PIN_A1), encoder1L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_A2), encoder2R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_A3), encoder3L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_A4), encoder4R, CHANGE);

  //
  Serial.println("Programm: Laufprogramm_SA19");
  Serial.println("Das Programm wurde erfolgreich initialisiert.");
  Serial.println("Info: Der H-Bridge Controller kann über folgende Befehle bedient werden: \n");
  Serial.print("Info: forward  = Motor dreht vorwärts \n");
  Serial.print("Info: backward = Motor dreht rückwärts \n");
  Serial.print("Info: brake    = Motor bremst \n");
  Serial.print("Info: finish   = Programm wird beendet --> sichere Zustand wird hergestellt \n");
  Serial.print("Info: test     = Test Case wird gestarte um die Spungantwort des Systems aufzunehmen und im Serialmonitor auszugeben \n");

}   //Ende void setup

void loop()
{

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
  Serial.println(Vornesollwert);*/
  analogWrite(PWM_1, VLRegOut);

  /*Serial.print("Position Vorne Rechts = ");
  Serial.println(encoder2Pos);
  Serial.print("Sollposition Vorne Rechts = ");
  Serial.println(Hintensollwert);*/
  analogWrite(PWM_2, VRRegOut);

  /*Serial.print("Position Hinten Links = ");
  Serial.println(encoder3Pos);
  Serial.print("Sollposition Hinten Links = ");
  Serial.println(Hintensollwert);*/
  analogWrite(PWM_3, HLRegOut);

  /*Serial.print("Position Hinten Rechts = ");
  Serial.println(encoder4Pos);
  Serial.print("Sollposition Hinten Rechts = ");
  Serial.println(Vornesollwert);*/
  analogWrite(PWM_4, HRRegOut);

  /*if (encoder1Pos >= Vornesollwert-toleranz && encoder2Pos >= Hintensollwert-toleranz && encoder3Pos >= Hintensollwert-toleranz && encoder4Pos >= Vornesollwert-toleranz)
  {
    Vornesollwert = Vornesollwert+schritt;
    Hintensollwert = Hintensollwert+schritt;
  } //Beine bewegen wenn alle auf Position sind*/
  Vornesollwert+=schritt;
  Hintensollwert+=schritt;
}

/*  void encoderLinks(volatile long encoderArr[][5], volatile int i)
  {
    encoderArr[i][0] = digitalRead(encoderArr[i][1]);
    if((encoderArr[i][2] == LOW) && (encoderArr[i][0] == HIGH))
    {
      if (digitalRead(encoderArr[i][3]) == LOW)
      {
        encoderArr[i][4]++;
      }
      else
      {
        encoderArr[i][4]--;
      }
    }
    encoderArr[i][2] = encoderArr[i][0];
  }

  void encoderRechts(volatile long encoderArr[][5], volatile int i)
  {
    encoderArr[i][0] = digitalRead(encoderArr[i][1]);
    if((encoderArr[i][2] == LOW) && (encoderArr[i][0] == HIGH))
    {
      if (digitalRead(encoderArr[i][3]) == LOW)
      {
        encoderArr[i][4]--;
      }
      else
      {
        encoderArr[i][4]++;
      }
    }
    encoderArr[i][2] = encoderArr[i][0];
  }
*/
  //void encoder1L(volatile long encoderArr[][5], volatile int i) //ISR Encoderprogrammablauf Encoder 1
  void encoder1L()
  {
    //i = 0;
    //encoderLinks(encoderArr, i);
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

  void encoder2R() //ISR Encoderprogrammablauf Encoder 2
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

  void encoder3L() //ISR Encoderprogrammablauf Encoder 3
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

  void encoder4R() //ISR Encoderprogrammablauf Encoder 4
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
