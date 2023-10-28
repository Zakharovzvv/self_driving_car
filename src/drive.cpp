#include <Arduino.h>
#include <Header.h>
#include <Console.h>
#include <IR_Sensor.h>

// Функция для управления двигателями
void drive(int L, int R, int interval = 0)
{
    digitalWrite(MOTOR_L_DIRECTION_PIN, L > 0 ? HIGH : LOW); // Управляем направлением левого мотора
    analogWrite(MOTOR_L_SPEED_PIN, abs(L));                  // Управляем скоростью левого мотора
    digitalWrite(MOTOR_R_DIRECTION_PIN, R > 0 ? HIGH : LOW); // Управляем направлением правого мотора
    analogWrite(MOTOR_R_SPEED_PIN, abs(R));                  // Управляем скоростью правого мотора

    delay(interval);
}

void left() {
  drive(-V, V); delay(1000);
  while (analogRead(A0) > 500) drive(-V, V);
  while (analogRead(A0) < 500) drive(-V, V);
  while (analogRead(A1) > 500) drive(-V, V);
}

void right() {
  drive(V, -V); delay(1000);
  while (analogRead(A0) > 500) drive(V, -V);
}

/// Функция для поворота
void turn(Direction direction,int linesCount)
{
    bool lastColor;     
    bool currentColor;     
    float speedGain=1.5;  

    int speedMotorL= direction == LEFT ? baseSpeed *(-1)*speedGain : baseSpeed*speedGain;
    int speedMotorR=speedMotorL*(-1);
    int IRSensor=direction == LEFT ? IR_SENSOR_R_PIN : IR_SENSOR_L_PIN;
 #if !DEBUG
    console("baseSpeed",baseSpeed);
   console("LS",speedMotorL,"RS",speedMotorR,"IRSensor",IRSensor);   
 #endif  
    
    currentColor = isSensorOnBlack(getIRSensorValue(IRSensor));
    lastColor=currentColor;
    int linesCrossed=currentColor==true?1:0; // Переменная для подсчета количества линий, которые пересек датчик
    
    drive(speedMotorL, speedMotorR);
    
    do
    {
        currentColor= isSensorOnBlack(getIRSensorValue(IRSensor));
        if (currentColor != lastColor)
        {
            linesCrossed++;
            lastColor=currentColor;
        };
 #if !DEBUG

        console("currentColor",currentColor,"lastColor",lastColor);
        console("linesCrossed",linesCrossed);
        delay(5000);
 #endif  

    } while (linesCrossed < linesCount);
    drive(0, 0);
}