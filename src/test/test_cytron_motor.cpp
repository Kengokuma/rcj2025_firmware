#include <Arduino.h>

#include "CytronMotorDriver.h"
#include "micro_geared_motor.hpp"

MicroGearedMotor motor(11, 12, 20, 21);
// Configure the motor driver.
CytronMD motor1(PWM_PWM, 0, 1);   // PWM 1A = Pin 0, PWM 1B = Pin 1.
CytronMD motor2(PWM_PWM, 2, 3); // PWM 2A = Pin 2, PWM 2B = Pin 3.
CytronMD motor3(PWM_PWM, 4, 5); // PWM 3A = Pin 4, PWM 3B = Pin 5.
CytronMD motor4(PWM_PWM, 6, 7); // PWM 4A = Pin 6, PWM 4B = Pin 7.

// The setup routine runs once when you press reset.
void setup() {
    motor.begin();
    motor.stop();
}

// The loop routine runs over and over again forever.
void loop() {
    
  motor1.setSpeed(128);  // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(-128); // Motor 2 runs backward at 50% speed.
  motor3.setSpeed(-128);    // Motor 3 runs forward at 50% speed.
  motor4.setSpeed(128);    // Motor 4 runs backward at 50% speed.
  delay(1000);

  motor1.setSpeed(255);  // Motor 1 runs forward at full speed.
  motor2.setSpeed(-255); // Motor 2 runs backward at full speed.
  motor3.setSpeed(-255);    // Motor 3 runs forward at full speed.
  motor4.setSpeed(255);    // Motor 4 runs backward at full speed.
  delay(1000);

  motor1.setSpeed(0); // Motor 1 stops.
  motor2.setSpeed(0); // Motor 2 stops.
  motor3.setSpeed(0); // Motor 3 stops.
  motor4.setSpeed(0); // Motor 4 stops.
  delay(1000);

  motor1.setSpeed(-128); // Motor 1 runs backward at 50% speed.
  motor2.setSpeed(128);  // Motor 2 runs forward at 50% speed.
  motor3.setSpeed(128); // Motor 3 runs backward at 50% speed.
  motor4.setSpeed(-128);  // Motor 4 runs forward at 50% speed.
  delay(1000);

  motor1.setSpeed(-255); // Motor 1 runs backward at full speed.
  motor2.setSpeed(255);  // Motor 2 runs forward at full speed.
  motor3.setSpeed(255); // Motor 3 runs backward at full speed.
  motor4.setSpeed(-255);  // Motor 4 runs forward at full speed.
  delay(1000);

  motor1.setSpeed(0); // Motor 1 stops.
  motor2.setSpeed(0); // Motor 2 stops.
  motor3.setSpeed(0); // Motor 3 stops.
  motor4.setSpeed(0); // Motor 4 stops.
  delay(1000);
}