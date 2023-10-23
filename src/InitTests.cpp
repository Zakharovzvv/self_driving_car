#include <Arduino.h>

#include <Drive.h>
#include <Header.h>
#include <Preg.h>
#include <Servo_Motor.h>
#include <UZ_Sensor.h>
#include <IR_Sensor.h>
#include <InitTests.h>

//################## tests ########################################
void check()
{
    drive(baseSpeed, baseSpeed, 500);
    drive(0, 0, 0);
    getDistance();
    moveServo(CLOSE);
    moveServo(OPEN);
}
void testServo(){
        if (getDistance() < distanceToTakeBotle){
         moveServo(CLOSE); //Берем банку
         delay(2000);
          moveServo(CLOSE); //Отпускаем банку
    }
}
//################################################################
