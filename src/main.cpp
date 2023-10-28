#define DEBUG 0

#if DEBUG
#include "avr8-stub.h"
#else
#include <Console.cpp>
#endif


#include <Arduino.h>

#include <Drive.h>
#include <Header.h>
#include <Preg.h>
#include <Servo_Motor.h>
#include <UZ_Sensor.h>
#include <IR_Sensor.h>
#include <InitTests.h>



const int MOTOR_L_DIRECTION_PIN = 2;
const int MOTOR_L_SPEED_PIN = 3;
const int MOTOR_R_DIRECTION_PIN = 4;
const int MOTOR_R_SPEED_PIN = 5;
// Устанавливаем номер пина для ультразвукового датчика
const int UZ_F_TRIGGER_PIN = 9;
const int UZ_F_ECHO_PIN = 10;
// Устанавливаем номер пина для сервопривода
const int SERVO_PIN = 13;
// Устанавливаем номера пинов для датчиков линии
const int IR_SENSOR_L_PIN = A0;
const int IR_SENSOR_R_PIN = A1;

const float SOUND_SPEED = 0.034; // Скорость звука см/сек

const int PIN_MAX_BIT = 250;     // Устанавливаем максимальную разрядность выхода на который подключены моторы
const int UZ_MAX_DISTANCE = 200; // Устанавливаем максимальное расстояние до объекта

// Инициализация переменных
int blackWhiteBorder= 500; //Значение ИК датчика, выше которого считаем, что он набелом, ниже на черном
int baseSpeed = 60; // Базовая скорость моторов
float pregK = 0.3;  // Коэффициент чувствительности П регулятора
int minIRL = 500, minIRR = 500, maxIRL = 600, maxIRR = 600;
int step = 0;
int crossCount = 0; // Количество пройденных перекрестков
int servoOpenPosition=30; //Угол сервопривода в открытом положении
int servoClosePosition=90; //Угол сервопривода в закрытом положении
int baseDelay=500; //Задержка выполнения шага
int distanceToTakeBotle = 6; // Расстояние до банки при котором нужно закрыть сервопривод (взять банку)
int distanceToCheckBotle=30; //Расстояние до банки, стоящей на перекрестке чтобы определить, что банка есть


// Функция настройки начальных параметров
void setup()
{
    pinMode(MOTOR_L_DIRECTION_PIN, OUTPUT); // Устанавливаем режимы работы пинов
    pinMode(MOTOR_L_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_R_DIRECTION_PIN, OUTPUT);
    pinMode(MOTOR_R_SPEED_PIN, OUTPUT);
    pinMode(UZ_F_TRIGGER_PIN, OUTPUT);
    pinMode(UZ_F_ECHO_PIN, INPUT);
    pinMode(IR_SENSOR_L_PIN, INPUT);
    pinMode(IR_SENSOR_R_PIN, INPUT);
    pinMode(SERVO_PIN, OUTPUT);

    initServo();

    // initialize GDB stub
#if DEBUG
    debug_init();
#else
    Serial.begin(9600); // Only using Serial when not debugging!
#endif
}

void loop()
{

   //  preg(); //Едем по линии
    testServo();
    // //Обрабатываем перекрестки
    //  if (isOnCros())
    //  {
    //      crossCount++;
    //      driveCross();
    //  };
    // //Если увидели банку - берем ее

    //     while (true)
    //     {
    //         /* code */
    //     }
        
    // }
}

