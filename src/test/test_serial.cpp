#include <Arduino.h>

void setup() {
  // Set up code here
  Serial.begin(9600);
}

void loop() {
  // Main loop code here
  Serial.println("Hello, World!");
  delay(1000); // Wait for a second before the next print
}