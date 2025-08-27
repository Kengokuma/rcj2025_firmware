#include <Arduino.h>
#include "micro_geared_motor.hpp"

MicroGearedMotor motor(11, 12, 20, 21);

void setup() {
  motor.begin();
}

void loop() {
  motor.set_speed(127);
  delay(1000);
  motor.stop();
  delay(1000);
  motor.set_speed(-128);
  delay(1000);
  motor.stop();
  delay(1000);
}