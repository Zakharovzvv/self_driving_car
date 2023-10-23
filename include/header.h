#ifndef HEADER_H
#define HEADER_H
#include <Arduino.h>
#include <Servo.h>   // Подключение библиотеки для управления сервоприводами

extern int V;
extern int min1;
extern int max1;
extern int min2;
extern int max2;

// Устанавливаем номера пинов для управления моторами
extern const int MOTOR_L_DIRECTION_PIN = 2;
extern const int MOTOR_L_SPEED_PIN = 3;
extern const int MOTOR_R_DIRECTION_PIN = 4;
extern const int MOTOR_R_SPEED_PIN = 5;
// Устанавливаем номер пина для ультразвукового датчика
extern const int UZ_F_TRIGGER_PIN = 9;
extern const int UZ_F_ECHO_PIN = 10;
// Устанавливаем номер пина для сервопривода
extern const int SERVO_PIN = 13;
// Устанавливаем номера пинов для датчиков линии
extern const int IR_SENSOR_L_PIN = A0;
extern const int IR_SENSOR_R_PIN = A1;

extern const float SOUND_SPEED = 0.034; // Скорость звука см/сек

extern const int PIN_MAX_BIT = 250;     // Устанавливаем максимальную разрядность выхода на который подключены моторы
extern const int UZ_MAX_DISTANCE = 200; // Устанавливаем максимальное расстояние до объекта

// Инициализация переменных
extern int blackWhiteBorder; //Значение ИК датчика, выше которого считаем, что он набелом, ниже на черном
extern int baseSpeed; // Базовая скорость моторов
extern float pregK ; // Коэффициент чувствительности П регулятора
extern int minIRL, minIRR , maxIRL, maxIRR ;
extern int step;
extern int crossCount; // Количество пройденных перекрестков
extern int servoOpenPosition; //Угол сервопривода в открытом положении
extern int servoClosePosition; //Угол сервопривода в закрытом положении
extern int baseDelay; //Задержка выполнения шага
extern int distanceToTakeBotle ; // Расстояние до банки при котором нужно закрыть сервопривод (взять банку)
extern int distanceToCheckBotle; //Расстояние до банки, стоящей на перекрестке чтобы определить, что банка есть

// Определение направлений для функции поворота
enum Direction
{
    LEFT,
    RIGHT
};

enum ServoState
{
    OPEN,
    CLOSE 
};

Servo servo;

#endif