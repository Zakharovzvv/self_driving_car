#include <Arduino.h>
#include <Header.h>
#include <Drive.h>

void right() {
  drive(V, -V); delay(1000);
  while (analogRead(A0) > 500) drive(V, -V);
}
