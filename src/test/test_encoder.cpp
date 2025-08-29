#include <Arduino.h>

#include "pio_encoder.h"

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
    encoder1.begin();
    encoder2.begin();
    encoder3.begin();
    encoder4.begin();
    Serial.begin(115200);
    while(!Serial);
}

// The loop routine runs over and over again forever.
void loop() {
  print_encoder_values();
  delay(100);
}

