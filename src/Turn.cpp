#include <Arduino.h>
#include <Drive.h>
#include <Header.h>
#include <Console.h>
#include <IR_Sensor.h>


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