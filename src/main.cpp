#include <Arduino.h>
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "encoder.h"

QuadratureEncoder encoder(5,6,10);

void setup() {
  // stdio_init_all();
  Serial.begin(115200);
  encoder.begin();
  Serial.print("init\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  Serial.printf(">raw:%f",encoder.get_raw());
  Serial.printf(">angle:%f",encoder.get_angle());
  Serial.printf(">total:%f",encoder.get_total_angle());
}