#include<Arduino.h>
#include<PID_v1.h>

#define PWM_1 13
#define DIR_1 21

int speedHome = 127; //Homegeschwindigkeit
int direction = HIGH; //allgemeine Drehrichtung Beine, vorwärts

void setup()
{
  pinMode (PWM_1, OUTPUT);
  pinMode (DIR_1, OUTPUT);

  Serial.begin (9600);
  while(!Serial) //warten bis Serialport verbindet
  {
    delay(1);
  }
  Serial.println("Ready");
  delay(2000);
  digitalWrite (DIR_1, direction); //Homingsequenz
  analogWrite (PWM_1, speedHome);
  delay(5000);

}

void loop()
{

}
