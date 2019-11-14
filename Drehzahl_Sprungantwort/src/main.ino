//Dieses Programm wurde anhand der Musterlösung des Drehzahlermittlungsprogrammes von Patrick Zellweger aus dem Mechatronikpraktikum Praktikum 7/2019 erstellt und für die eigene Anwendung angepasst!

#include <Arduino.h>
#include <Ticker.h>

//Funktionsdeklaration
void encoder1(); //Funktion zur Encoderabfrage
void testFunction(); //Funktion zur Sprungantwortermittlung
void drehzahlBerechnen(); //Funktion zur Drehzahlberechnung
Ticker testSprung(testFunction, zykluszeit, 0, MICROS_MICROS);

//Pinbelegung
#define PIN_A1 5 //Input A-Signal Encoder VL
#define PIN_B1 6 //Input B-Signal Encoder VL
#define PIN_I1 9 //Input I-Signal Encoder VL
#define PWM_1 13 //PWM-Ausgang für Motor VL
#define DIR_1 21 //Richtungsangabe für Motor VL

//Definition Variablen für Encoderauswertung
volatile unsigned long encoder1Pos = 0; //Encoder Value auf 0 Stellen
volatile unsigned long encoder1PosAlt = 0; //alter Encoderwert
volatile int encoderPinA1Last = LOW; //Nötige Voreinstellungen für das Encoder Programm
volatile bool n1 = LOW; //liest A-Signal des Encoders aus
const int aufloesungEncoder = 360; //Impulse pro Umdrehung
const unsigned long groesseEncoder = (unsigned long) pow(2,sizeof(encoder1Pos)*8); //Variable für Overflowüberwachung
unsigned int encoder1Differenz; //Anzahl Impulse seit letzter Abfrage -> für Drehzahlberechnung
double istDrehzahl;

//Variablen für Sprungantwortgenerierung
unsigned long time = 0; //in us
unsigned long timeAlt = 0; //in us
unsigned long testTime = 500000; //Testzeit für Aufnahem der Sprungantwort in Microsekunden
const int zykluszeit = 5000; //in Mikrosekunden

//Definition der State-Variablen
enum state{idle, drehzahl, test, stop};
state = idle;

//Testvariable
int speedTest = 60; //Variable Motorinput für Überprüfung Drehzahlberechnung
int speedSprung = 200; //Variable Motorinput für Aufnahme der Sprungantwort

//Definition der Stringhandlingsvariablen
bool serialPrintFlag = false; // FLAG welcher verwendet wird, damit Ausgaben im Serialmonitor nur einmal geschrieben werden
String serialString; // Variable in welchem der String aus dem Serialmonitor gespeichert wird
const String strDrehzahl = String("drehzahl"); // konstante Stringvariable, wird zur Überprüfung der Eingabe im Serialmonitor verwendet
const String strTest = String("test"); // konstante Stringvariable, wird zur Überprüfung der Eingabe im Serialmonitor verwendet
const String strStop = String("stop"); // konstante Stringvariable, wird zur Überprüfung der Eingabe im Serialmonitor verwendet

void setup()
{
  //Pinmodes definieren
  pinMode (PIN_A1, INPUT);
  pinMode (PIN_B1, INPUT);
  pinMode (PIN_I1, INPUT);
  pinMode (PWM_1, OUTPUT);
  pinMode (DIR_1, OUTPUT);

  //Motoren initialisieren
  analogWrite (PWM_1, LOW);   //Geschwindigkeit Motor VL auf Null setzen
  digitalWrite (DIR_1, HIGH);  //Drehrichtung Motor VL vorwärts setzen

  //ISR-Implementierung
  attachInterrupt(digitalPinToInterrupt(PIN_A1), encoder1, CHANGE);     //ISR-Definierung für Abtastung Encoder 1

  //Initialiserung des SerialMonitors
  Serial.begin (9600);
  while(!Serial) //warten bis Serialport verbindet
  {
    delay(1);
  }

  //Ausgabe der Programmlogik
  Serial.println("Status: Das Programm wurde erfolgreich initialisiert");
  Serial.println("Status: Das Programm wird folgendermassen bedient");
  Serial.println("Info: drehzahl = Programm gibt Drehzahl von Testvariable aus");
  Serial.println("Info: test = Test Case wird gestarte um die Spungantwort des Systems aufzunehmen und im Serialmonitor auszugeben");
  Serial.println("Info: stop = Programm wird beendet");

} //Ende void setup

void loop()
{
  //leeren der Stringvariable
  serialString = "";

  // Wenn im Serialbuffer bytes vorhanden sind, werden die Daten im Serialbuffer verarbeitet
  if (Serial.available() > 0)
  {
    serialString = Serial.readString(); // Lese String aus dem Serialbuffer
    serialString = serialString.substring(0,serialString.length()-1); // Umschaltzeichen welches durch die Eingabe (Enter drücken) hinzugefügt wurde muss entfernt werden
  }

  //Statemachine
  switch(state)
  {
    // State idle:
    case idle :
      // Schreibe Output einmal beim Eintritt in den State:
      if(serialPrintFlag == false)
      {
        Serial.println("Bitte geben Sie eine Aktion ein (drehzahl, test, stop)!");
        serialPrintFlag = true;
      }
      // State wechseln basierend auf vorliegender Eingabe im Serialmonitor
      if(serialString.equals(strDrehzahl))
      {
        state = drehzahl;
        serialPrintFlag = false;
        encoder1PosAlt = encoder1Pos;
      }
      else if(serialString.equals(strTest))
      {
        state = test;
        serialPrintFlag = false;
      }
      else if(serialString.equals(strStop))
      {
        state = stop;
        serialPrintFlag = false;
      }
      break;

    // State Drehzahl
    case drehzahl :
    //Ausführen Drehzahltest
    if(serialPrintFlag == false)
    {
      analogWrite(PWM_1, speedTest);
      Serial.print("Aktuelle Drehzahl = ");
      Serial.println(istDrehzahl);
      serialPrintFlag = true;
    }
    // Wechsel zu State Test oder Stop
    if(serialString.equals(strTest) || serialString.equals(strStop))
    {
      if(serialString.equals(strTest))
      {
        state = test;
        serialPrintFlag = false;
        analogWrite(PWM_1, LOW);
      }
      else
      {
        state = stop;
        serialPrintFlag = false;
        analogWrite(PWM_1, LOW);
      }
    }
    break;

    //State Test
    case test :
    // Starten des Testvorgangs:
    if(serialPrintFlag == false)
    {
      timeAlt = micros();
      testSprung.resume();
      analogWrite(PWM_1, speedSprung); // Sprung setzen --> ACHTUNG: nicht bis ans Maximum!
      serialPrintFlag = true;
    }
    testSprung.update(); //testSprung updaten
    // Beenden des Testvorgangs:
    if ((micros() - timeAlt) >= testTime)
    {
      state = idle;
      testSprung.stop();
      analogWrite(PWM_1, LOW); // Motorinput = 0
      serialPrintFlag = false;
    }
    break;

    //Case stop
    case stop :
    // Schreibe Output und Ausführen der brake-Funktion einmal beim Eintritt:
    if(serialPrintFlag == false)
    {
      Serial.println("Programm wurde beendet");
      serialPrintFlag_ = true;
    }
    break;
  }
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

void berechneDrehzahl();
{
  // Counter Differenz:
  // Der Overflow der Countervariable wird  folgendermassen überwacht:
  // Ist die Differenz zwischen dem alten und dem neuen Wert grösser als die Hälfte des Wertebereichs des Datentyps, wird von einnem Overflow ausgegeangen.
  // Die Annahme beruht drauf, dass eine solch grosse counterDifferenz nicht auftreten kann.
  // Das gleiche Prinzip wird beim Underflow angewendet.
  // Underflow Fall Rückwärts
  if((encoder1Pos > encoder1PosAlt) && ((encoder1Pos -encoder1PosAlt) > groesseEncoder/2))
  {
    encoder1Differenz = (unsigned int)(groesseEncoder - 1 - encoder1Pos + encoder1PosAlt);
  }
  // Kein Underflow Rückwärts
  else if ((encoder1Pos < encoder1PosAlt) && ((encoder1PosAlt - encoder1Pos) < groesseEncoder/2))
  {
    encoder1Differenz = (unsigned int)(encoder1PosAlt - encoder1Pos);
  }
  // Overflow Vorwärts
  else if ((encoder1Pos < encoder1PosAlt) && ((encoder1PosAlt - encoder1Pos) > groesseEncoder/2))
  {
    encoder1Differenz = (unsigned int)(groesseEncoder - 1 - encoder1PosAlt + encoder1Pos);
  }
  // Kein Overflow Vorwärts
  else if ((encoder1Pos > encoder1PosAlt) && ((encoder1Pos - encoder1PosAlt) < groesseEncoder/2))
  {
    encoder1Differenz = (unsigned int)(encoder1Pos - encoder1PosAlt);
  }
  // Keine Bewegung
  else if (encoder1Pos == encoder1PosAlt)
  {
    encoder1Differenz = 0;
  }
  else
  {
    Serial.println("Fehler Encoderauswertung");
  }
  // aktuellen Counter für nächste berechnung speichern:
  encoder1PosAlt = encoder1Pos;
  // Frequenz und Drehzahl Encoder:
  istDrehzahl = ((((double)encoder1Differenz/(double)zykluszeit)/(double)aufloesungEncoder)*60000000);
}

void testFunction()
{
  berechneDrehzahl();
  Serial.print("Drehzahl : ");
  Serial.println(istDrehzahl);
  Serial.print("Zeit : ");
  Serial.println(micros() - timeAlt);
}
