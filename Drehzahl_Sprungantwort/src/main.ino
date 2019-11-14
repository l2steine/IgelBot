#include <Arduino.h>

//Funktionsdeklaration
void encoder1(); //Funktion zur Encoderabfrage
void drehzahlBerechnen(); //Funktion zur Drehzahlberechnung

//Pinbelegung
#define PIN_A1 5 //Input A-Signal Encoder VL
#define PIN_B1 6 //Input B-Signal Encoder VL
#define PIN_I1 9 //Input I-Signal Encoder VL
#define PWM_1 13 //PWM-Ausgang für Motor VL
#define DIR_1 21 //Richtungsangabe für Motor VL

//Definition Variablen für Encoderauswertung
volatile int encoder1Pos = 0; //Encoder Value auf 0 Stellen
volatile int encoderPinA1Last = LOW; //Nötige Voreinstellungen für das Encoder Programm
volatile int n1 = LOW; //liest A-Signal des Encoders aus

//Variablen für Sprungantwortgenerierung
unsigned long timeAlt = 0; //in us
unsigned long testTime = 5000000; //Testzeit für Aufnahme der Sprungantwort in Microsekunden

//Variable für Drehzahlberechnung, Encoder gibt 360 Impulse pro Umdrehung
unsigned long timeAlt2 = 0; //in us
double istDrehzahl;

//Sprungeingang
int sprung = 150; //Variable Motorinput für Aufnahme der Sprungantwort

void setup()
{
  //Pinmodes definieren
  pinMode (PIN_A1, INPUT);
  pinMode (PIN_B1, INPUT);
  pinMode (PIN_I1, INPUT);
  pinMode (PWM_1, OUTPUT);
  pinMode (DIR_1, OUTPUT);

  //Motoren initialisieren
  analogWrite (PWM_1, LOW); //Geschwindigkeit Motor VL auf Null setzen
  digitalWrite (DIR_1, HIGH); //Drehrichtung Motor VL vorwärts setzen

  //ISR-Implementierung
  attachInterrupt(digitalPinToInterrupt(PIN_A1), encoder1, CHANGE); //ISR-Definierung für Abtastung Encoder 1

  //Initialiserung des SerialMonitors
  Serial.begin (9600);
  while(!Serial) //warten bis Serialport verbindet
  {
    delay(1);
  }
  Serial.println("Sprung wird in 3 Sekunden erzeugt!");
  delay(3000);

  //Sprungantwort aufnehmen
  analogWrite(PWM_1, sprung); //Eingabe Motorsprung
  timeAlt = micros();
  while ((micros()-timeAlt) <= testTime)
  {
    berechneDrehzahl();
    Serial.print("Zeit = ");
    Serial.print((micros()-timeAlt));
    Serial.print(" Drehzahl = ");
    Serial.println(istDrehzahl);
  }
  analogWrite(PWM_1, 0); //Motor abschalten

} //Ende void setup

void loop()
{
  //kein Loop; Programm wird nur zur Sprungantwortermittlung benötigt
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

void berechneDrehzahl()
{
  timeAlt2 = micros();
  encoder1Pos = 0;
  if(encoder1Pos == 90)
  {
    istDrehzahl = 0.25*6000000/(micros()-timeAlt2);
  }
}
