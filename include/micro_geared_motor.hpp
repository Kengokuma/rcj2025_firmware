#ifndef MICRO_GEARED_MOTOR_HPP
#define MICRO_GEARED_MOTOR_HPP

#include <Arduino.h>
#include "pio_encoder.h"

class MicroGearedMotor {
public:
  MicroGearedMotor(int pwmPin, int dirPin, int encA, int encB);
  void begin();
  void set_speed(int speed);
  void stop();
  void move_to_angle(int angle);

private:
  int pwmPin_;
  int dirPin_;
  PioEncoder encoder_;
  const int MAX_SPEED = 255;
  const int ENCODER_STEPS_PER_REV = 20; // Adjust based on your motor's specifications
  const int POSITION_TOLERANCE = 2;     // Tolerance for position control
};

#endif
