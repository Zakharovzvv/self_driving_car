#include <Arduino.h>
#include <Servo.h>

Servo servo; 

int V=200;
float K=1;
int min1=500,min2=500, max1=600, max2=600;
float UZDSENCE=0.01723;
int step=0;

void setup() {
  servo.attach(13);
  servo.write(30);
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
  Serial.begin(9600);
}

void move(int R, int L){
  if (L>0){
    digitalWrite(2,1); analogWrite(3,L);
    }
    else {
    digitalWrite(2,0); analogWrite(3,abs(L));
    }
  if (R>0){
    digitalWrite(4,1); analogWrite(5,R);
    }
    else {
    digitalWrite(4,0); analogWrite(5,abs(R));
    } 
}

void normalize(int& d1,int& d2){
if (d1<min1) min1=d1;
if (d2<min2) min2=d2;
if (d1>max1) max1=d1;
if (d2>max2) max2=d2;
d1=map(analogRead(A0),min1,max1,0,1000);
d2=map(analogRead(A1),min2,max2,0,1000);
}
//preg
void preg(){
int d1=analogRead(A0);
int d2=analogRead(A1);

normalize(d1,d2);

int E=d1-d2;
int U=K*E;
int M1=V+U; M1=constrain(M1,-250,250);
int M2=V-U; M2=constrain(M2,-250,250);
move(M1,M2);
}

bool isOnCross(){
  return (analogRead(A0)<500) && (analogRead(A1)<500); //Если показания обоих датчиков <500, то оба на черном
}

void start(){
  move (V,V);
  delay(1000);
  step++;
}

void turnLeft(){
  move(V,-V); delay(500);
  while(analogRead(A0)>500){move(-V,V);}
  while(analogRead(A1)>500){move(-V,V);}

}

void turnRight(){
  move(V,-V); delay(500);
  while(analogRead(A1)>500){move(V,-V);}
 // while(analogRead(A0)<500){move(V,-V);}

}

int uzdF(){
  digitalWrite(9,0);
  delayMicroseconds(2);
  digitalWrite(9,1);
  delayMicroseconds(10);
  digitalWrite(9,0);
  return UZDSENCE*pulseIn(10,1);
}

void banka(){
  if (uzdF()<6){
    move(0,0);delay(1000);
    for (int i = 30; i < 80; i++)
    { servo.write(i); delay(10); }
      move(-V,-V); delay(500);
      turnLeft();
      while (analogRead(A2)<400) preg();
      move(0,0); delay (1000);
      for (int i = 80; i > 30; i--)
    { servo.write(i); delay(10); }
    move(-V,-V); delay(1000);
    turnLeft();
      step++;
      move(0,0);while(1){};
  }
  else
  {  preg(); }
  
}

void cross(){
  if (isOnCross()==true){
    move(V,V); delay(500);
    move(0,0); delay(500);
    turnLeft();
    move(0,0); delay(500);
    if (uzdF()<30){
      banka();     
    }
    else
    {
      turnLeft();turnLeft();
      move(0,0);
    }
    
  }
  else preg();
  step++;
}

void loop() {
 switch (step)
 {
 case 0: start(); break;
  case 1: cross(); break;
   case 2: banka(); break;  
  }
}


void print(){

}
