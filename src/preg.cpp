#include <Arduino.h>
#include <header.h>
#include <go.h>

void preg() {
  int d1 = analogRead(A0);
  int d2 = analogRead(A1);
  if (d1 < min1) min1 = d1;
  if (d2 < min2) min2 = d2;
  if (d1 > max1) max1 = d1;
  if (d2 > max2) max2 = d2;
  d1 = map(analogRead(A0), min1, max1, 0, 1000);
  d2 = map(analogRead(A1), min2, max2, 0, 1000);
  int E = d1 - d2;
  float K = 0.4;
  int M1 = V + E * K; M1 = constrain(M1, -255, 255);
  int M2 = V - E * K; M2 = constrain(M2, -255, 255);
  go(M1, M2);
}
