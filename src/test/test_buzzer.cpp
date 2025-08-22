#include <Arduino.h>

void setup() {
  // Set up code here
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(26, OUTPUT);
}

void loop() {
  // Main loop code here
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
  tone(26, 1000, 100); // Generate a tone on GPIO26
  delay(1000); // Wait for a second
  digitalWrite(LED_BUILTIN, LOW); // Turn the LED off
  tone(26, 500, 100);
  delay(1000); // Wait for a second
}