#include <Arduino.h>
#include "pio_encoder.h"

PioEncoder encoder(10); // encoder is connected to GPIO10 and GPIO11

void setup() {
  encoder.begin();
  Serial.begin(115200);
  while(!Serial);

}

void loop() {
  Serial.println(encoder.getCount());
  delay(10);
}