#define DEBUG 0

#if DEBUG
#include "avr8-stub.h"
#endif

#if !DEBUG
#include <Console.cpp>
#endif

#include <Arduino.h>
#include <Servo.h>   // Подключение библиотеки для управления сервоприводами
#include <NewPing.h> // Подключение библиотеки для управления датчиком расстояния

// Устанавливаем номера пинов для управления моторами
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

// Создание объекта для управления сервоприводом
Servo servo;
NewPing sonar(9, 10); // NewPing setup of pins and maximum distance.

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

/*
// Функция для нормализации значений с датчиков
void normalize(int& leftIRValue, int& rightIRValue) {

    // Обновляем минимальные и максимальные значения
    minIRL = min(minIRL, leftIRValue);
    minIRR = min(minIRR, rightIRValue);
    maxIRL = max(maxIRL, leftIRValue);
    maxIRR = max(maxIRR, rightIRValue);
    // Преобразовываем значения датчиков
    leftIRValue = map(analogRead(IR_SENSOR_L_PIN), minIRL, maxIRL, 0, 1000);
    rightIRValue = map(analogRead(IR_SENSOR_R_PIN), minIRR, maxIRR, 0, 1000);
}
*/
// Функция для нормализации значений ИК датчика
int getIRSensorValue(int sensor)
{
    int irValue = analogRead(sensor); // Читаем значение датчика
                                      // Обновляем минимальные и максимальные значения
    if (sensor == IR_SENSOR_L_PIN)
    {
        minIRL = min(minIRL, irValue);
        maxIRL = max(maxIRL, irValue);

        return map(irValue, minIRL, maxIRL, 0, 1000); // Преобразовываем значения датчиков
    }
    else
    {
        minIRR = min(minIRR, irValue);
        maxIRR = max(maxIRR, irValue);

        return map(irValue, minIRR, maxIRR, 0, 1000); // Преобразовываем значения датчиков
    };
}

// Проверяем, находится ли датчик на черной линии
bool isSensorOnBlack(int sensorValue)
{
    return sensorValue < blackWhiteBorder;
}
// Проверяем, что оба датчика на черной линии. Значит мы на перекрестке
bool isOnCros()
{
    return isSensorOnBlack(getIRSensorValue(IR_SENSOR_L_PIN)) && isSensorOnBlack(getIRSensorValue(IR_SENSOR_R_PIN));
}

// Функция для управления двигателями
void drive(int L, int R, int interval = 0)
{
    digitalWrite(MOTOR_L_DIRECTION_PIN, L > 0 ? HIGH : LOW); // Управляем направлением левого мотора
    analogWrite(MOTOR_L_SPEED_PIN, abs(L));                  // Управляем скоростью левого мотора
    digitalWrite(MOTOR_R_DIRECTION_PIN, R > 0 ? HIGH : LOW); // Управляем направлением правого мотора
    analogWrite(MOTOR_R_SPEED_PIN, abs(R));                  // Управляем скоростью правого мотора

    delay(interval);
}

// Функция для управления движением по линии
void preg()
{
    int d1 = getIRSensorValue(IR_SENSOR_L_PIN); // Читаем значение левого датчика
    int d2 = getIRSensorValue(IR_SENSOR_R_PIN); // Читаем значение правого датчика
 
    int E = d1 - d2;     // Ошибка- разность показаний датчиков
    float U = E * pregK; // Корректировка ошибки умножением на коэффициент

    int leftMotorSpeed = baseSpeed + U;  // Корректируем скорость и направление левого мотора
    int rightMotorSpeed = baseSpeed - U; // Корректируем скорость и направление правого мотора

    leftMotorSpeed = constrain(leftMotorSpeed, -PIN_MAX_BIT, PIN_MAX_BIT);   // Приводим значение скорости к диапазону +/- MOTOR_SPEED
    rightMotorSpeed = constrain(rightMotorSpeed, -PIN_MAX_BIT, PIN_MAX_BIT); // Приводим значение скорости к диапазону +/- MOTOR_SPEED

    drive(leftMotorSpeed, rightMotorSpeed); // Вызываем функцию движения и передаем ей скорректированные значения скоростей мотров

    // Выводим значения датчиков в серийный порт для отладки
        console("Left Sensor: ", d1, "Right Sensor: ", d2);
}

// Функция для обработки перекрестка
void driveCross()
{
    drive(0, 0, 500);
    while (isOnCros())
    {
        drive(baseSpeed, baseSpeed);
    }

    // drive(baseSpeed, baseSpeed, baseDelay); // Двигаемся вперед
    // drive(0, 0, baseDelay);                 // Останавливаем робота
}

void uzdImpulse(int pin)
{
}
// Функция для считывания данных с ультразвукового датчика
int getForwardDistance()
{
    // digitalWrite(UZ_F_TRIGGER_PIN, LOW);  // Сбрасываем датчик
    // delayMicroseconds(2);    //

    // digitalWrite(UZ_F_TRIGGER_PIN, HIGH); // Отправляем импульс до объекта
    // delayMicroseconds(10);
    // digitalWrite(UZ_F_TRIGGER_PIN, LOW);

    // long duration =pulseIn(UZ_F_ECHO_PIN, HIGH); //Получаем время прохождения импульса до объекта и обратно

    //    int distance = duration * SOUND_SPEED/2 ; // Считаем расстояние до объекта
    int distance = sonar.ping_cm(); // Считаем расстояние до объекта

#if !DEBUG
    console("d=", distance);
#endif

    return distance;
}
// Функция открытия/закрытия сервопривода
void moveServo(ServoState state)
{

    if (state == OPEN)
    {
        for (int i = servoOpenPosition; i < servoClosePosition; i++)
        {
            servo.write(i); // Устанавливаем позицию сервопривода
            delay(10);      // Ждем между шагами
#if !DEBUG
 //           console("servo=", i);
#endif
        }
    }
    else
    {
        for (int i = servoClosePosition; i > servoOpenPosition; i--)
        {
            servo.write(i); // Устанавливаем позицию сервопривода
            delay(10);      // Ждем между шагами
#if !DEBUG
 //           console("servo=", i);
#endif
        }
    }
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

// Функция для обработки препятствий
void takeBanka()
{

    // Едем пока не приблизимся к банке на расстояние захвата
    while (getForwardDistance() > distanceToTakeBotle)
    {
        preg();
    };

    drive(0, 0, baseDelay); // Останавливаем робота

    moveServo(CLOSE); //Берем банку

    drive(-baseSpeed, -baseSpeed, baseDelay); // Двигаемся назад
    turn(LEFT,3);                                    // Поворачиваем влево
}
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
        if (getForwardDistance() < distanceToTakeBotle){
         moveServo(CLOSE); //Берем банку
         delay(4000);
          moveServo(OPEN); //Отпускаем банку
          delay(4000);
    }
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

