#include <Arduino.h>

#define PWM_1 13
#define DIR_1 21

void setup()
{
  pinMode (PWM_1, OUTPUT);
  pinMode (DIR_1, OUTPUT);
  digitalWrite (DIR_1, HIGH); //Homingsequenz
  analogWrite (PWM_1, LOW);
  delay(500);
  Serial.begin(9600);
  delay (2000);
  Serial.println("Ready");
  delay(2000);
  Serial.println("funktioniert");
  analogWrite(PWM_1, 63);
  delay(5000);
  analogWrite(PWM_1, LOW);
}

void loop()
{

}
