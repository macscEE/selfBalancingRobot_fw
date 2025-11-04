#include <Arduino.h>


#define LED 13

void setup() {
  pinMode(LED, OUTPUT);
  
}

void loop() {
  digitalWrite(LED, LOW);
  delay(50);
  digitalWrite(LED, HIGH);
  delay(50);
}