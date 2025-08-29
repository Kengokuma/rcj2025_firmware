#include <Arduino.h>
#include "rescue_kit.hpp"

RescueKit kit(8, 9, 20, 27);

void setup() {
  kit.begin();
}

void loop() {
  kit.deploy(1, true);
}