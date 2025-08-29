#include <Arduino.h>

#include "CytronMotorDriver.h"
#include "micro_geared_motor.hpp"
#include "pio_encoder.h"

MicroGearedMotor micro_motor(8, 9, 20, 21);
// Configure the motor driver.
CytronMD motor1(PWM_PWM, 0, 1); // PWM 1A = Pin 0, PWM 1B = Pin 1.
CytronMD motor2(PWM_PWM, 2, 3); // PWM 2A = Pin 2, PWM 2B = Pin 3.
CytronMD motor3(PWM_PWM, 4, 5); // PWM 3A = Pin 4, PWM 3B = Pin 5.
CytronMD motor4(PWM_PWM, 6, 7); // PWM 4A = Pin 6, PWM 4B = Pin 7.

PioEncoder encoder1(10);
PioEncoder encoder2(14);
PioEncoder encoder3(16);
PioEncoder encoder4(18);

void print_encoder_values() {
    Serial.print("Encoder 1: ");
    Serial.print(encoder1.getCount());
    Serial.print(" Encoder 2: ");
    Serial.print(encoder2.getCount());
    Serial.print(" Encoder 3: ");
    Serial.print(encoder3.getCount());
    Serial.print(" Encoder 4: ");
    Serial.println(encoder4.getCount());
}

// The setup routine runs once when you press reset.
void setup() {
    micro_motor.begin();
    delay(1000);
    micro_motor.stop();
    delay(1000);
    encoder1.begin();
    encoder2.begin();
    encoder3.begin();
    encoder4.begin();
    Serial.begin(115200);
}

// The loop routine runs over and over again forever.
void loop() {
    
  motor1.setSpeed(128);  // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(-128); // Motor 2 runs backward at 50% speed.
  motor3.setSpeed(-128);    // Motor 3 runs forward at 50% speed.
  motor4.setSpeed(128);    // Motor 4 runs backward at 50% speed.
  delay(1000);

  print_encoder_values();

  motor1.setSpeed(255);  // Motor 1 runs forward at full speed.
  motor2.setSpeed(-255); // Motor 2 runs backward at full speed.
  motor3.setSpeed(-255);    // Motor 3 runs forward at full speed.
  motor4.setSpeed(255);    // Motor 4 runs backward at full speed.
  delay(1000);

  print_encoder_values();

  motor1.setSpeed(0); // Motor 1 stops.
  motor2.setSpeed(0); // Motor 2 stops.
  motor3.setSpeed(0); // Motor 3 stops.
  motor4.setSpeed(0); // Motor 4 stops.
  delay(1000);

  print_encoder_values();

  motor1.setSpeed(-128); // Motor 1 runs backward at 50% speed.
  motor2.setSpeed(128);  // Motor 2 runs forward at 50% speed.
  motor3.setSpeed(128); // Motor 3 runs backward at 50% speed.
  motor4.setSpeed(-128);  // Motor 4 runs forward at 50% speed.
  delay(1000);

  print_encoder_values();

  motor1.setSpeed(-255); // Motor 1 runs backward at full speed.
  motor2.setSpeed(255);  // Motor 2 runs forward at full speed.
  motor3.setSpeed(255); // Motor 3 runs backward at full speed.
  motor4.setSpeed(-255);  // Motor 4 runs forward at full speed.
  delay(1000);

  print_encoder_values();

  motor1.setSpeed(0); // Motor 1 stops.
  motor2.setSpeed(0); // Motor 2 stops.
  motor3.setSpeed(0); // Motor 3 stops.
  motor4.setSpeed(0); // Motor 4 stops.
  delay(1000);

  print_encoder_values();
}

