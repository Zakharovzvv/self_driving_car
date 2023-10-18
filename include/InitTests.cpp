#include <main.cpp>

//################## tests ########################################
void check()
{
    drive(baseSpeed, baseSpeed, 500);
    drive(0, 0, 0);
    getForwardDistance();
    moveServo(CLOSE);
    moveServo(OPEN);
}
void testServo(){
        if (getForwardDistance() < UZD_OBSTACLE_DISTANCE){
         moveServo(CLOSE); //Берем банку
         delay(2000);
          moveServo(CLOSE); //Отпускаем банку
    }
}
//################################################################
