#include <Arduino.h>
int uzdL() {
  digitalWrite(7, 0);
  delayMicroseconds(2);
  digitalWrite(7, 1);
  delayMicroseconds(10);
  digitalWrite(7, 0);
  return 0.01723 * pulseIn(8, 1);
}
