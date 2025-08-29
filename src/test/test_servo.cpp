#include <Arduino.h>
#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(27);
}

void loop() {
  myServo.write(180);
  delay(1000);
  myServo.write(90);
  delay(1000);
  myServo.write(0);
  delay(1000);
  myServo.write(90);
  delay(1000);
}