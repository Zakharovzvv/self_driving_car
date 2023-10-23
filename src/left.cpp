#include <Arduino.h>
#include <header.h>
#include <go.h>

void left() {
  go(-V, V); delay(1000);
  while (analogRead(A0) > 500) go(-V, V);
  while (analogRead(A0) < 500) go(-V, V);
  while (analogRead(A1) > 500) go(-V, V);
}
