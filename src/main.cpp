#include <Arduino.h>  // Подключение стандартной библиотеки Arduino
#include <Servo.h>    // Подключение библиотеки для работы с сервоприводом

Servo servo;  // Объявление объекта сервопривода

// Глобальные переменные
int V=200;                    // Стандартная скорость двигателя
float K=1;                    // Коэффициент для регулировки скорости
int min1=500, min2=500, max1=600, max2=600;  // Пределы для нормализации датчиков
float UZDSENCE=0.01723;       // Константа для ультразвукового датчика
int step=0;                   // Текущий этап в программе

void setup() {
  servo.attach(13);           // Присоединение сервопривода к пину 13
  servo.write(30);            // Установка начального положения сервопривода
  // Настройка пинов
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,INPUT);
  pinMode(10,INPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  Serial.begin(9600);        // Начало серийного соединения для отладки
}

// Функция для управления двигателями
void move(int R, int L){
  // Управление левым двигателем
  if (L>0){
    digitalWrite(2,1);
    analogWrite(3,L);
  }
  else {
    digitalWrite(2,0);
    analogWrite(3,abs(L));
  }
  // Управление правым двигателем
  if (R>0){
    digitalWrite(4,1);
    analogWrite(5,R);
  }
  else {
    digitalWrite(4,0);
    analogWrite(5,abs(R));
  } 
}

// Функция для нормализации показаний датчиков
void normalize(int& d1,int& d2){
  if (d1<min1) min1=d1;
  if (d2<min2) min2=d2;
  if (d1>max1) max1=d1;
  if (d2>max2) max2=d2;
  d1=map(analogRead(A0),min1,max1,0,1000);
  d2=map(analogRead(A1),min2,max2,0,1000);
}

void preg(){
  int d1=analogRead(A0);
  int d2=analogRead(A1);
  normalize(d1,d2);
  int E=d1-d2;  // Разница между датчиками
  int U=K*E;    // Управляющий сигнал на основе разницы
  int M1=V+U; 
  int M2=V-U;
  M1=constrain(M1,-250,250);  // Ограничение скорости
  M2=constrain(M2,-250,250);
  move(M1,M2);  // Применение скоростей к моторам
}

// Функция проверки нахождения на перекрестке
bool isOnCross(){
  return (analogRead(A0)<500) && (analogRead(A1)<500); 
}

// Начальное движение робота
void start(){
  move(V,V); 
  delay(1000);
  step++;
}

// Функция поворота налево
void turnLeft(){
  move(V,-V); delay(500);
  while(analogRead(A0)>500){move(-V,V);}
  while(analogRead(A1)>500){move(-V,V);}
}

// Функция поворота направо
void turnRight(){
  move(V,-V); delay(500);
  while(analogRead(A1)>500){move(V,-V);}
}

// Чтение данных с ультразвукового датчика
int uzdF(){
  digitalWrite(9,0);
  delayMicroseconds(2);
  digitalWrite(9,1);
  delayMicroseconds(10);
  digitalWrite(9,0);
  return UZDSENCE*pulseIn(10,1);
}

// Функция обработки препятствий
void banka(){
  if (uzdF()<6){
    move(0,0); delay(1000);
    for (int i = 30; i < 80; i++) {
      servo.write(i); delay(10);
    }
    move(-V,-V); delay(500);
    turnLeft();
    while (analogRead(A2)<400) preg();
    move(0,0); delay(1000);
    for (int i = 80; i > 30; i--) {
      servo.write(i); delay(10);
    }
    move(-V,-V); delay(1000);
    turnLeft();
    step++;
    move(0,0);
    while(1){};  // Зацикливание программы
  }
  else {
    preg();
  }
}

// Функция обработки перекрестка
void cross(){
  if (isOnCross()==true){
    move(V,V); delay(500);
    move(0,0); delay(500);
    turnLeft();
    move(0,0); delay(500);
    if (uzdF()<30){
      banka();     
    }
    else {
      turnLeft();turnLeft();
      move(0,0);
    }
  }
  else preg();
  step++;
}

// Основной цикл управления
void loop() {
  switch (step) {
    case 0: start(); break;
    case 1: cross(); break;
    case 2: banka(); break;
  }
}

void print(){
  // Функция вывода данных (пока пустая)
}
