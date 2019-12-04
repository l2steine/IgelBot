#include <Arduino.h>
#include <PID_v1.h>
#include <Wifi101.h>
#include <SPI.h>

void encoder1L();
void encoder2R();
void encoder3L();
void encoder4R();

//Pinbelegung
#define PIN_A1 9 //Input A-Signal Encoder VL
#define PIN_B1 6 //Input B-Signal Encoder VL
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

int speedHome = 100; //PWM-Output für Homespeed
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

//Statemachinevariablen
enum stateMode {idle, home, start, stop};
stateMode state = idle;

//Variablen für Stringverarbeitung initialisieren
String serialStringInput;
const String stringHome = String("home");
const String stringStart = String("start");
const String stringStop = String("stop");
bool flag = false;

//Variablen für Steuerung per Wifi
const char ssid[] = "Hedgi";
const char pass[] = "Semesterarbeit";
String request;
bool wifiStatus = LOW;
WiFiClient client;
WiFiServer server(80);
const char graphischeSeite[] ="<!DOCTYPE html>\n"
                              "<html>\n"
                              "  <head>\n"
                              "    <meta http-equiv=\"content-type\" content=\"text/html; charset=UTF-8\">\n"
                              "    <title>Semesterarbeit 2019 Steiner</title>\n"
                              "  </head>\n"
                              "  <body style=\"background-color:#427C98;\">\n"
                              "    <h1 style=\"color: white; font-family: Bahnschrift; text-align: center;\">autonom agierender Igel-Roboter</h1>\n"
                              "    <h2 style=\"color: white; font-family: Bahnschrift; text-align: center;\"><u>Programmsteuerung</u></h2>\n"
                              "    <p style=\"text-align: center;\">\n"
                              "      <a href=\\home\\><button type=\"button\">Home</button></a>\n"
                              "      <a href=\\start\\><button type=\"button\">Start</button></a> \n"
                              "      <a href=\\stop\\><button type=\"button\">Stop</button></a></p>\n"
                              "  </body>\n"
                              "</html>\n";

void setup()
{
  WiFi.setPins(8,7,4,2); //Wlanmodul aktivieren
  wifiStatus = WiFi.beginAP(ssid); //AccessPoint einrichten
  server.begin(); //Webserver starten

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

  //Motoren initialisieren
  analogWrite (PWM_1, LOW); //Geschwindigkeit Motor VL auf Null setzen
  analogWrite (PWM_2, LOW); //Geschwindigkeit Motor VR auf Null setzen
  analogWrite (PWM_3, LOW); //Geschwindigkeit Motor HL auf Null setzen
  analogWrite (PWM_4, LOW); //Geschwindigkeit Motor HR auf Null setzen
  digitalWrite (DIR_0, HIGH); //Drehrichtung aller Motoren vorwärts

  Serial.begin (9600);
  while(!Serial) //warten bis Serialport verbindet
  {
    delay(1);
  }
  Serial.println("Serial Monitor verbunden!");

  //Reglermodus einstellen
  VLReg.SetMode(AUTOMATIC);
  VRReg.SetMode(AUTOMATIC);
  HLReg.SetMode(AUTOMATIC);
  HRReg.SetMode(AUTOMATIC);

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
    serialStringInput.trim(); //entfernt die Leerzeichen nach der Eingabe
    Serial.flush();
  }

/*
  //Steuerung per WebServer
  WiFiClient client = server.available();

  while(client.connected())
	{
		if(client.available())
    {
      char c = client.read();
      if (c == '\n')
      {
        if(request.length() == 0)
        {
          client.print(graphischeSeite);
          client.println();
        }
        else
        {
          request = "";
        }
      }
      else if(c != '\r')
      {
        request += c;
      }
      if (request.startsWith("GET /home"))
      {
        state = home;
        flag = false;
      }
    }
*/
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
        while(encoder1Pos<990) //Wert muss angepasst werden
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
        while(encoder2Pos<960) //Wert muss angepasst werden
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
        while(encoder3Pos<805) //Wert muss angepasst werden
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
        while(encoder4Pos<1060) //Wert muss angepasst werden
        {
          encoder4R();
        }
        analogWrite (PWM_4, LOW); //Startposition erreicht
        Serial.println("Homing beendet");

        //ISR für Encoderabfrage definieren
        attachInterrupt(digitalPinToInterrupt(PIN_A1), encoder1L, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_A2), encoder2R, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_A3), encoder3L, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_A4), encoder4R, CHANGE);

        //Encoderwerte auf Startposition nullen
        encoder1Pos = 0;
        encoder2Pos = 0;
        encoder3Pos = 0;
        encoder4Pos = 0;

      }
      //Nach Homing wird Startbefehl oder Stopbefehl erwartet
      if(serialStringInput.equals(stringStart))
      {
        state = start;
        flag = false;
      }
      else if(serialStringInput.equals(stringStop))
      {
        state = stop;
        flag = false;
      }/*
      if (request.startsWith("GET /start"))
      {
        state = start;
        flag = false;
      }
      else if (request.startsWith("GET /stop"))
      {
        state = stop;
        flag = false;
      }
*/
      break;

    case start:
      while(state == start)
      {
        if(flag == false)
        {
          Serial.println("Igel läuft vorwärts!");
          flag = true;
        }
        //Geschwindigkeit bestimmen mit Hilfe des delays (Werte zwischen 5 und 10 sinnvoll)
        delay(10);

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
        Vornesollwert+=schritt;
        Hintensollwert+=schritt;

        //Stringwert abfragen
        if(Serial.available()>0)
        {
          serialStringInput = Serial.readString();
          serialStringInput.trim(); //entfernt alle überflüssigen Leerzeichen welche bei der Befehlseingabe mitgeliefert werden
          Serial.flush();
          if(serialStringInput.equals(stringStop))
          {
            state = stop;
            flag = false;
          }
        }/*
        if(client.available())
        {
          char c = client.read();
          if (c == '\n')
          {
            if(request.length() == 0)
            {
              client.print(graphischeSeite);
              client.println();
            }
            else
            {
              request = "";
            }
          }
          else if(c != '\r')
          {
            request += c;
          }
          if (request.startsWith("GET /stop"))
          {
            state = stop;
            flag = false;
          }
        }

      }*/
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
      }/*
      if (request.startsWith("GET /start"))
      {
        state = start;
        flag = false;
      }*/
      break;
    }

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
