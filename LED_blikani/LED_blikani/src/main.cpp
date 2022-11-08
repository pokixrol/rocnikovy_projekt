#include <Arduino.h>

//Blikání led na tlačítko

byte led = 14;
byte button = 12;

void setup() {
  pinMode(led, OUTPUT);
  pinMode(button, INPUT);

}

void loop() {
  if(digitalRead(button)){digitalWrite(led, HIGH);}
  else{digitalWrite(led,LOW);}

}