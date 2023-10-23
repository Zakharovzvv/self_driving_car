#include <Arduino.h>
#include <Header.h>
#include <Drive.h>

void left() {
  drive(-V, V); delay(1000);
  while (analogRead(A0) > 500) drive(-V, V);
  while (analogRead(A0) < 500) drive(-V, V);
  while (analogRead(A1) > 500) drive(-V, V);
}
