#include <Arduino.h>
#include <Servo.h>
#include "micro_geared_motor.hpp"

Servo myServo;
MicroGearedMotor motor(11, 12, 20, 21);

void setup() {
  myServo.attach(27);
  motor.begin();
  motor.stop();
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