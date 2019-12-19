//Libraries einfügen
#include <Arduino.h>
#include <PID_v1.h>

//Encoderfunktionen deklarieren
void encoder1L();
void encoder2R();
void encoder3L();
void encoder4R();

//Pinbelegung
#define PIN_A1 6 //Input A-Signal Encoder VL
#define PIN_B1 9 //Input B-Signal Encoder VL
#define PIN_I1 0 //Input I-Signal Encoder VL
#define PWM_1 13 //PWM-Ausgang für Motor VL
#define PIN_A2 21 //Input A-Signal Encoder VR
#define PIN_B2 5 //Input B-Signal Encoder VR
#define PIN_I2 1 //Input I-Signal Encoder VR
#define PWM_2 12 //PWM-Ausgang für Motor VR
#define PIN_A3 19 //Input A-Signal Encoder HL
#define PIN_B3 18 //Input B-Signal Encoder HL
#define PIN_I3 17 //Input I-Signal Encoder HL
#define PWM_3 11 //PWM-Ausgang für Motor HL
#define PIN_A4 16 //Input A-Signal Encoder HR
#define PIN_B4 15 //Input B-Signal Encoder HR
#define PIN_I4 14 //Input I-Signal Encoder HR
#define PWM_4 10 //PWM-Ausgang Motor HR
#define DIR_0 20//Richtungsangabe für alle Motoren

//Deklaration der Reglerin- und outputs (muessen double sein!)
double VLRegIn;
double VLRegOut;
double VRRegIn;
double VRRegOut;
double HLRegIn;
double HLRegOut;
double HRRegIn;
double HRRegOut;

double Kp=1.5, Ki=0, Kd=0.1; //PID Regleranteile

//Sollposition der Beine
double vorneSollwert = 180;
double hintenSollwert = 0;

//PID-Positionsregler definieren (Momentanwert, Regelwert, Sollwert)
PID VLReg(&VLRegIn, &VLRegOut, &vorneSollwert, Kp, Ki, Kd, DIRECT);
PID VRReg(&VRRegIn, &VRRegOut, &hintenSollwert, Kp, Ki, Kd, DIRECT);
PID HLReg(&HLRegIn, &HLRegOut, &hintenSollwert, Kp, Ki, Kd, DIRECT);
PID HRReg(&HRRegIn, &HRRegOut, &vorneSollwert, Kp, Ki, Kd, DIRECT);

int speedHome = 100; //PWM-Output für Homespeed
int schritt = 5; //Winkelerhöhung pro Zyklus in Grad

//Variablen für Encoderverarbeitung
int i1, i2, i3, i4; //Variablen zur Speicherung des I-Signals
volatile long encoder1Pos = 0; //Encodervalues auf 0 Stellen
volatile long encoder2Pos = 0;
volatile long encoder3Pos = 0;
volatile long encoder4Pos = 0;
volatile long encoderPinA1Last = LOW; //nötige Voreinstellungen für Encoderfunktion
volatile long encoderPinA2Last = LOW;
volatile long encoderPinA3Last = LOW;
volatile long encoderPinA4Last = LOW;
volatile int n1 = LOW; //Speicherplatz A-Signal
volatile int n2 = LOW;
volatile int n3 = LOW;
volatile int n4 = LOW;

//Statemachinevariablen
enum stateMode {idle, home, start, stop};
stateMode state = idle;

//Variablen für Stringverarbeitung initialisieren
String serialStringInput;
const String stringHome = String("home");
const String stringStart = String("start");
const String stringStop = String("stop");
bool flag = false;

void setup()
{
  //Pins definieren
  pinMode (PIN_A1, INPUT);
  pinMode (PIN_B1, INPUT);
  pinMode (PIN_I1, INPUT);
  pinMode (PWM_1, OUTPUT);
  pinMode (PIN_A2, INPUT);
  pinMode (PIN_B2, INPUT);
  pinMode (PIN_I2, INPUT);
  pinMode (PWM_2, OUTPUT);
  pinMode (PIN_A3, INPUT);
  pinMode (PIN_B3, INPUT);
  pinMode (PIN_I3, INPUT);
  pinMode (PWM_3, OUTPUT);
  pinMode (PIN_A4, INPUT);
  pinMode (PIN_B4, INPUT);
  pinMode (PIN_I4, INPUT);
  pinMode (PWM_4, OUTPUT);
  pinMode (DIR_0, OUTPUT);

  //PWM-Outputbegrenzung des Reglers
  VLReg.SetOutputLimits(0,255);
  VRReg.SetOutputLimits(0,255);
  HLReg.SetOutputLimits(0,255);
  HRReg.SetOutputLimits(0,255);

  //Reglermodus einstellen
  VLReg.SetMode(AUTOMATIC);
  VRReg.SetMode(AUTOMATIC);
  HLReg.SetMode(AUTOMATIC);
  HRReg.SetMode(AUTOMATIC);

  //Motoren initialisieren
  analogWrite (PWM_1, LOW); //Geschwindigkeit Motoren auf Null setzen
  analogWrite (PWM_2, LOW);
  analogWrite (PWM_3, LOW);
  analogWrite (PWM_4, LOW);
  digitalWrite (DIR_0, HIGH); //Drehrichtung aller Motoren vorwärts

  Serial.begin(9600);
  while(!Serial) //warten bis Serialport verbindet
  {
    delay(1);
  }
  Serial.println("Serial Monitor verbunden!");

  //Erklärung der Programmsteuerung
  Serial.println("Laufprogramm_SA19");
  Serial.println("Das Programm wurde erfolgreich initialisiert.");
  Serial.println("Es kann über folgende Befehle gesteuert werden:");
  Serial.println("home  = Programm führt Homingfunktion aus");
  Serial.println("start = startet Laufbewegung");
  Serial.println("stop  = stoppt Laufbewegung");

} //Ende void setup

void loop()
{
  //Steuerung per Serialmonitor
  serialStringInput = "";
  //Serialmonitor abfragen
  if(Serial.available()>0)
  {
    serialStringInput = Serial.readString();
    serialStringInput.trim(); //entfernt alle Leerzeichen nach den eingegebenen Zeichen
    Serial.flush();
  }

  //Case der Statemachine bestimmen
  switch(state)
  {
    case idle:
      //Information ausgeben
      if(flag == false)
      {
        Serial.println("Bevor das Laufprogramm gestartet oder gestoppt werden kann, muessen die Beine gehomet werden.");
        Serial.println("Bitte geben Sie den Befehl home ein.");
        flag = true;
      }
      if(serialStringInput.equals(stringHome))
      {
        state = home;
        flag = false;
      }
      break;

    case home:
      if(flag == false)
      {
        Serial.println("Homing beginnt!");
        flag = true;

        //Homing vorne links
        analogWrite (PWM_1, speedHome);
        Serial.println("Homing vorne links...");
        while (i1 == 0) //Position kalibrieren
        {
          i1 = digitalRead(PIN_I1); //Indeximpuls für Referenzierung
        }
        while(encoder1Pos<875) //Winkelposition der Homingposition, muss angepasst werden wenn Encoder getauscht werden
        {
          encoder1L();
        }
        analogWrite (PWM_1, LOW); //Startposition erreicht

        //Homings vorne rechts
        analogWrite (PWM_2, speedHome);
        Serial.println("Homing vorne rechts...");
        while (i2 == 0) //Position kalibrieren
        {
          i2 = digitalRead(PIN_I2); //Indeximpuls für Referenzierung
        }
        while(encoder2Pos<1010) //Winkelposition der Homingposition, muss angepasst werden wenn Encoder getauscht werden
        {
          encoder2R();
        }
        analogWrite (PWM_2, LOW); //Startposition erreicht

        //Homing hinten links
        analogWrite (PWM_3, speedHome);
        Serial.println("Homing hinten links...");
        while (i3 == 0) //Position kalibrieren
        {
          i3 = digitalRead(PIN_I3); //Indeximpuls für Referenzierung
        }
        while(encoder3Pos<1035) //Winkelposition der Homingposition, muss angepasst werden wenn Encoder getauscht werden
        {
          encoder3L();
        }
        analogWrite (PWM_3, LOW); //Startposition erreicht

        //Homing hinten rechts
        analogWrite (PWM_4, speedHome);
        Serial.println("Homing hinten rechts...");
        while (i4 == 0) //Position kalibrieren
        {
          i4 = digitalRead(PIN_I4); //Indeximpuls für Referenzierung
        }
        while(encoder4Pos<1020) //Winkelposition der Homingposition, muss angepasst werden wenn Encoder getauscht werden
        {
          encoder4R();
        }
        analogWrite (PWM_4, LOW); //Startposition erreicht
        Serial.println("Homing beendet");

        //Encoderwerte auf Startposition = 0 setzen
        encoder1Pos = 0;
        encoder2Pos = 0;
        encoder3Pos = 0;
        encoder4Pos = 0;

        //ISR's für Encoderabfrage definieren
        attachInterrupt(digitalPinToInterrupt(PIN_A1), encoder1L, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_A2), encoder2R, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_A3), encoder3L, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_A4), encoder4R, CHANGE);

      }
      //Nach Homing wird Startbefehl oder Stopbefehl erwartet
      if(serialStringInput.equals(stringStart))
      {
        state = start;
        flag = false;
      }
      break;

    case start:
      while(state == start)
      {
        if(flag == false)
        {
          Serial.println("Igel läuft vorwärts!");
          flag = true;
        }
        //Geschwindigkeit bestimmen mit Hilfe des delays (Werte zwischen 5 und 15 sinnvoll)
        delay(11);

        //Encoderpositionen als Reglereingänge definieren
        VLRegIn = encoder1Pos;
        VRRegIn = encoder2Pos;
        HLRegIn = encoder3Pos;
        HRRegIn = encoder4Pos;

        //Regelsystem updaten
        VLReg.Compute();
        VRReg.Compute();
        HLReg.Compute();
        HRReg.Compute();

        //Gschwindigkeit der Motoren anpassen
        analogWrite(PWM_1, VLRegOut);
        analogWrite(PWM_2, VRRegOut);
        analogWrite(PWM_3, HLRegOut);
        analogWrite(PWM_4, HRRegOut);

        //Beinposition um Schrittwert erhöhen
        vorneSollwert+=schritt;
        hintenSollwert+=schritt;

        //Stringwert abfragen
        if(Serial.available()>0)
        {
          serialStringInput = Serial.readString();
          serialStringInput.trim(); //entfernt alle Leerzeichen nach den eingegebenen Zeichen
          Serial.flush();
          if(serialStringInput.equals(stringStop))
          {
            state = stop;
            flag = false;
          }
        }
      }
      break;

    case stop:
      if(flag == false)
      {
        Serial.println("Laufvorgang wird beendet!");
        flag = true;

        //alle Motoren abschalten
        analogWrite(PWM_1, LOW);
        analogWrite(PWM_2, LOW);
        analogWrite(PWM_3, LOW);
        analogWrite(PWM_4, LOW);
      }
      if(serialStringInput.equals(stringStart))
      {
        state = start;
        flag = false;
      }
      break;
  } //Ende Statemachine
} //Ende void loop

  void encoder1L()
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
