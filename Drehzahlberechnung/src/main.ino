#include <Arduino.h>
#include <PID_v1.h>

void encoder1();

//Pinbelegung
#define PIN_A1 5 //Input A-Signal Encoder VL
#define PIN_B1 6 //Input B-Signal Encoder VL
#define PIN_I1 9 //Input I-Signal Encoder VL
#define PWM_1 13 //PWM-Ausgang für Motor VL
#define DIR_1 21 //Richtungsangabe für Motor VL

int i1; //A-Signal des Encoder für Bein VL

volatile long encoder1Pos = 0; //Encoder Values auf 0 Stellen
volatile long encoder1PosAlt = 0;
volatile int encoderPinA1Last = LOW;  //Nötige Voreinstellungen für das Encoder Programm
volatile int n1 = LOW;
volatile long time = 0;
volatile long timeAlt = LOW;
volatile int istDrehzahl;

int speedHome = 60;

void setup()
{
  pinMode (PIN_A1, INPUT);          //Pins definieren
  pinMode (PIN_B1, INPUT);
  pinMode (PIN_I1, INPUT);
  pinMode (PWM_1, OUTPUT);
  pinMode (DIR_1, OUTPUT);

  //Motoren initialisieren
  analogWrite (PWM_1, LOW);   //Geschwindigkeit Motor VL auf Null setzen
  digitalWrite (DIR_1, HIGH);  //Drehrichtung Motor VL vorwärts

  Serial.begin (9600);
  while(!Serial) //warten bis Serialport verbindet
  {
    delay(1);
  }
  Serial.println("Ready");
  delay(2000);

  analogWrite (PWM_1, speedHome); //Homingsequenz

  attachInterrupt(digitalPinToInterrupt(PIN_A1), encoder1, CHANGE);     //ISR-Definierung für Abtastung Encoder 1

}   //Ende void setup

void loop()
{
  Serial.print("aktuelle Drehzahl = ");
  Serial.println(istDrehzahl);
  delay(1000);
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
  time = micros();
  encoderPinA1Last = n1;
  istDrehzahl = (encoder1Pos-encoder1PosAlt)*60000000/(360*(time-timeAlt));
  timeAlt = time;
  encoder1PosAlt = encoder1Pos;

}
