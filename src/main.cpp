#define DEBUG 0

#if DEBUG
#include "avr8-stub.h"
#endif

#if !DEBUG
#include <Console.cpp>
#endif

#include <Arduino.h>
#include <Servo.h>   // Подключение библиотеки для управления сервоприводами

#include <Drive.h>
#include <Header.h>
#include <Preg.h>
#include <Servo_Motor.h>
#include <UZ_Sensor.h>
#include <IR_Sensor.h>
#include <InitTests.h>


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

    servo.attach(SERVO_PIN);          // Привязываем сервопривод к пину
    servo.write(servoOpenPosition); // Устанавливаем начальное положение сервопривода

    // initialize GDB stub
#if DEBUG
    debug_init();
#else
    Serial.begin(9600); // Only using Serial when not debugging!
#endif
}



//################## tests ########################################
void check()
{
    drive(baseSpeed, baseSpeed, 500);
    drive(0, 0, 0);
    getDistance();
    moveServo(CLOSE);
    moveServo(OPEN);
}

//################################################################
// Основной цикл программы
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

