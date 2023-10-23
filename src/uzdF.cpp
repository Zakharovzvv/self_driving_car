#include <Arduino.h>
int uzdF() {
  digitalWrite(9, 0);
  delayMicroseconds(2);
  digitalWrite(9, 1);
  delayMicroseconds(10);
  digitalWrite(9, 0);
  return 0.01723 * pulseIn(10, 1);
}
