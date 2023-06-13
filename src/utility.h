#ifndef UTILITY_H
#define UTILITY_H

#include "Arduino.h"
template<class T>
bool inRange(T val, T min, T max) {
    if (val>=min && val<=max) {
        return true;
    }
    else {
        return false;
    }
}
template <typename T>
void plot(String name, T val) {
  Serial.print(">");
  Serial.print(name);
  Serial.print(":");
  Serial.println(val);
}

#endif