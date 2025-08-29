#include "rescue_kit.hpp"

RescueKit::RescueKit(int pwmPin, int dirPin, int encPin, int servoPin)
    : pwmPin_(pwmPin), dirPin_(dirPin), encoder_(encPin), servoPin_(servoPin) {}

void RescueKit::begin() {
  pinMode(pwmPin_, OUTPUT);
  pinMode(dirPin_, OUTPUT);
  encoder_.begin();
  servo_.attach(servoPin_);
  Serial.begin(115200);
}

void RescueKit::set_speed(int speed) {
  if (speed > 0) {
    digitalWrite(dirPin_, HIGH);
  } else {
    digitalWrite(dirPin_, LOW);
  }
  analogWrite(pwmPin_, abs(speed));
}

void RescueKit::deploy(int num, bool is_left) {
  for (int i = 0; i < num; i++){
    long now_pos = encoder_.getCount();
    long error = target_angle - (encoder_.getCount() - now_pos);
    int count = 0;
    while (count < 1000) {
      error = target_angle - (encoder_.getCount() - now_pos);
      int speed = static_cast<int>((Kp * error) * max_speed);
      set_speed(speed);
      delay(1);
      Serial.print("Current Speed: ");
      Serial.println(speed);
      count++;
    }
    set_speed(0);
    Serial.print("Final Position: ");
    Serial.println(encoder_.getCount());
    delay(1000);
    if (is_left) {
      servo_.write(0);
    } else {
      servo_.write(180);
    }
    delay(1000);
    servo_.write(90);
    delay(1000);
  }
}