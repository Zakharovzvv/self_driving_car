#include <Drive.h>
#include <Header.h>
#include <Preg.h>
#include <Servo_Motor.h>
#include <UZ_Sensor.h>
#include <Turn.h>

// Функция для обработки препятствий
void takeBanka()
{

    // Едем пока не приблизимся к банке на расстояние захвата
    while (getDistance() > distanceToTakeBotle)
    {
        preg();
    };

    drive(0, 0, baseDelay); // Останавливаем робота

    moveServo(CLOSE); //Берем банку

    drive(-baseSpeed, -baseSpeed, baseDelay); // Двигаемся назад
    turn(LEFT,3);                                    // Поворачиваем влево
}