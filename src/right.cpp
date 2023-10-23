#include <Arduino.h>
#include <header.h>
#include <go.h>

void right() {
  go(V, -V); delay(1000);
  while (analogRead(A0) > 500) go(V, -V);
}
