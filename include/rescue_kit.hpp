#ifndef RESCUE_KIT_HPP
#define RESCUE_KIT_HPP

#include <Arduino.h>
#include <Servo.h>
#include "pio_encoder.h"

class RescueKit {
public:
  RescueKit(int pwmPin, int dirPin, int encPin, int servoPin);
  void begin();
  void set_speed(int speed);
  void deploy(int num, bool is_left);

private:
  int pwmPin_;
  int dirPin_;
  int encPin_;
  int servoPin_;
  static constexpr float Kp = 0.005;
  static constexpr long target_angle = 200;
  static constexpr int max_speed = 255;

  PioEncoder encoder_;
  Servo servo_;
};

#endif
