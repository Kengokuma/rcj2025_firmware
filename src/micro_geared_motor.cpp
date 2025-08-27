#include "micro_geared_motor.hpp"
#include "Arduino.h"

MicroGearedMotor::MicroGearedMotor(int pwmPin, int dirPin, int encA, int encB)
    : pwmPin_(pwmPin), dirPin_(dirPin), encoder_(encA, encB) {}

void MicroGearedMotor::begin() {
  pinMode(pwmPin_, OUTPUT);
  pinMode(dirPin_, OUTPUT);
  encoder_.begin();
}

void MicroGearedMotor::set_speed(int speed) {
  if (speed > 0) {
    digitalWrite(dirPin_, HIGH);
  } else {
    digitalWrite(dirPin_, LOW);
  }
  analogWrite(pwmPin_, abs(speed));
}

void MicroGearedMotor::stop() {
  digitalWrite(dirPin_, LOW);
  analogWrite(pwmPin_, 0);
}

void MicroGearedMotor::move_to_angle(int angle) {
  int current_position = encoder_.getCount();
  int target_position = angle * ENCODER_STEPS_PER_REV / 360;
  int error = target_position - current_position;

  while (abs(error) > POSITION_TOLERANCE) {
    set_speed(error > 0 ? MAX_SPEED : -MAX_SPEED);
    current_position = encoder_.getCount();
    error = target_position - current_position;
  }
  stop();
}
