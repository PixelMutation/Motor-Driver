#include <Arduino.h>
// #include "pico/stdio.h"
// #include "pico/stdlib.h"
// #include "encoder.h"

// QuadratureEncoder encoder(5,6,10);

// void setup() {
//   // stdio_init_all();
//   Serial.begin(115200);
//   delay(1000);
//   Serial.print("init start\n");
//   if (encoder.begin()!=-1)
//     Serial.print("init success\n");
//   else {
//     while (true){Serial.print("init fail");delay(100);}
//   }
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   delay(100);
//   // Serial.print("hi");
//   Serial.printf(">raw:%f\n",encoder.get_raw());
//   Serial.printf(">angle:%f\n",encoder.get_angle());
//   Serial.printf(">total:%f\n",encoder.get_total_angle());
// }