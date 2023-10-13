#define __USING_DEBUG_MODE__ 1

#if __USING_DEBUG_MODE__
#include "avr8-stub.h"
#define DEBUG_LED_PIN (13)
#endif

// Подключение стандартной библиотеки для Arduino
#include <Arduino.h>

// Подключение библиотеки для управления сервоприводами
#include <Servo.h>

// Устанавливаем номер пина для сервопривода
const int SERVO_PIN = 13;

// Устанавливаем номера пинов для управления моторами
const int MOTOR_L_DIRECTION_PIN = 2;
const int MOTOR_L_SPEED_PIN = 3;
const int MOTOR_R_DIRECTION_PIN = 4;
const int MOTOR_R_SPEED_PIN = 5;

// Устанавливаем номера пинов для датчиков линии
const int IR_SENSOR_LEFT_PIN = A0;
const int IR_SENSOR_RIGHT_PIN = A1;

// Устанавливаем номер пина для ультразвукового датчика
const int UZ_F_TRIGGER_PIN = 9;
const int UZ_F_ECHO_PIN = 10;

// Устанавливаем порог для датчика линии
const int IR_SENSOR_THRESHOLD = 500;

// Устанавливаем коэффициент для ультразвукового датчика
const float SOUND_SPEED = 0.01723;

// Устанавливаем расстояния для датчика
const int UZD_OBSTACLE_DISTANCE = 6;
const int UZD_CROSS_DISTANCE = 30;

// Устанавливаем позиции сервопривода
const int SERVO_OPEN_POSITION = 30;
const int SERVO_CLOSE_POSITION = 80;

// Устанавливаем стандартную задержку
const int STANDARD_DELAY = 500;

// Устанавливаем максимальную разрядность выхода на который подключены моторы
const int PIN_MAX_BIT = 250;

// Создание объекта для управления сервоприводом
Servo servo;

// Инициализация переменных
int baseSpeed = 200; // Базовая скорость моторов
float pregK = 0.5;   // Коэффициент чувствительности П регулятора
int minIRL = 500, minIRR = 500, maxIRL = 600, maxIRR = 600;
int step = 0;
int crossCount = 0; // Количество пройденных перекрестков

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

#if !__USING_DEBUG_MODE__
// Первый шаблон для завершения рекурсии
template <typename T>
void debugPrintInternal(const char *name, T value)
{
    Serial.print(name);
    Serial.print(": ");
    Serial.println(value);
}

// Рекурсивный шаблон для обработки переменного количества аргументов
template <typename T, typename... Args>
void debugPrintInternal(const char *name, T value, Args... args)
{
    Serial.print(name);
    Serial.print(": ");
    Serial.print(value);
    Serial.print(", ");
    debugPrintInternal(args...);
}

// Функция для вывода отладочной информации
template <typename... Args>
void debugPrint(Args... args)
{
    debugPrintInternal(args...);
    Serial.println(); // добавляем новую строку после вывода
}
#endif
// Функция настройки начальных параметров
void setup()
{
    servo.attach(SERVO_PIN);                // Привязываем сервопривод к пину
    servo.write(SERVO_OPEN_POSITION);       // Устанавливаем начальное положение сервопривода
    pinMode(MOTOR_L_DIRECTION_PIN, OUTPUT); // Устанавливаем режимы работы пинов
    pinMode(MOTOR_L_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_R_DIRECTION_PIN, OUTPUT);
    pinMode(MOTOR_R_SPEED_PIN, OUTPUT);
    pinMode(UZ_F_TRIGGER_PIN, INPUT);
    pinMode(UZ_F_ECHO_PIN, OUTPUT);
    pinMode(IR_SENSOR_LEFT_PIN, INPUT);
    pinMode(IR_SENSOR_RIGHT_PIN, INPUT);

    // drive(baseSpeed, baseSpeed,STANDARD_DELAY*3); //Начинаем движение без П регулятора

    // initialize GDB stub
    debug_init();
#if !__USING_DEBUG_MODE__
    Serial.begin(9600); // Only using Serial when not debugging!
#endif                  // Инициализация серийного порта для вывода отладочной информации
}

// Функция для управления двигателями
void drive(int R, int L, int interval = 0)
{
    digitalWrite(MOTOR_L_DIRECTION_PIN, L > 0 ? HIGH : LOW); // Управляем направлением левого мотора
    analogWrite(MOTOR_L_SPEED_PIN, abs(L));                  // Управляем скоростью левого мотора
    digitalWrite(MOTOR_R_DIRECTION_PIN, R > 0 ? HIGH : LOW); // Управляем направлением правого мотора
    analogWrite(MOTOR_R_SPEED_PIN, abs(R));                  // Управляем скоростью правого мотора

    delay(interval);
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
    leftIRValue = map(analogRead(IR_SENSOR_LEFT_PIN), minIRL, maxIRL, 0, 1000);
    rightIRValue = map(analogRead(IR_SENSOR_RIGHT_PIN), minIRR, maxIRR, 0, 1000);
}
*/
// Функция для нормализации значений с датчиков
int getIRSensorValue(int sensor)
{
    int irValue = analogRead(sensor); // Читаем значение датчика
                                      // Обновляем минимальные и максимальные значения
    if (sensor == IR_SENSOR_LEFT_PIN)
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
    return sensorValue < IR_SENSOR_THRESHOLD;
}

// Проверяем, находятся ли оба датчика на черной линии
bool isOnCross()
{
    return isSensorOnBlack(getIRSensorValue(IR_SENSOR_LEFT_PIN)) && isSensorOnBlack(getIRSensorValue(IR_SENSOR_RIGHT_PIN));
}

// Функция для поворота
void turn(Direction direction)
{
    // Если направление влево
    if (direction == LEFT)
    {
        drive(baseSpeed, -baseSpeed, STANDARD_DELAY); // Поворачиваем влево

        // Пока левый датчик не обнаружит черную линию
        while (!isSensorOnBlack(getIRSensorValue(IR_SENSOR_LEFT_PIN)))
        {
            drive(-baseSpeed, baseSpeed);
        }
        // Пока правый датчик не обнаружит черную линию
        while (!isSensorOnBlack(getIRSensorValue(IR_SENSOR_RIGHT_PIN)))
        {
            drive(-baseSpeed, baseSpeed);
        }
    }
    else
    {                                                 // Если направление вправо
        drive(-baseSpeed, baseSpeed, STANDARD_DELAY); // Поворачиваем вправо

        // Пока правый датчик не обнаружит черную линию
        while (!isSensorOnBlack(getIRSensorValue(IR_SENSOR_RIGHT_PIN)))
        {
            drive(baseSpeed, -baseSpeed);
        }
    }
}

void cross();

// Функция для управления движением по линии
void preg()
{
    int d1 = getIRSensorValue(IR_SENSOR_LEFT_PIN);  // Читаем значение левого датчика
    int d2 = getIRSensorValue(IR_SENSOR_RIGHT_PIN); // Читаем значение правого датчика

    int E = d1 - d2;   // Ошибка- разность показаний датчиков
    int U = E * pregK; // Корректировка ошибки умножением на коэффициент

    int leftMotorSpeed = baseSpeed + U;  // Корректируем скорость и направление левого мотора
    int rightMotorSpeed = baseSpeed - U; // Корректируем скорость и направление правого мотора

    leftMotorSpeed = constrain(leftMotorSpeed, -PIN_MAX_BIT, PIN_MAX_BIT);   // Приводим значение скорости к диапазону +/- MOTOR_SPEED
    rightMotorSpeed = constrain(rightMotorSpeed, -PIN_MAX_BIT, PIN_MAX_BIT); // Приводим значение скорости к диапазону +/- MOTOR_SPEED

    drive(leftMotorSpeed, rightMotorSpeed); // Вызываем функцию движения и передаем ей скорректированные значения скоростей мотров

    // Выводим значения датчиков в серийный порт для отладки
    //    debugPrint("Left Sensor: ", d1, "Right Sensor: ", d2);
}

void uzdImpulse(int pin)
{
    digitalWrite(pin, LOW);  // Отправляем сигнал LOW на пин
    delayMicroseconds(2);    // Ждем немного
    digitalWrite(pin, HIGH); // Отправляем сигнал HIGH на пин
    delayMicroseconds(10);   // Ждем немного
    digitalWrite(pin, LOW);  // Отправляем сигнал LOW на пин
}

// Функция для считывания данных с ультразвукового датчика
int getForwardDistance()
{
    uzdImpulse(UZ_F_TRIGGER_PIN);
    int distance = SOUND_SPEED * pulseIn(UZ_F_ECHO_PIN, HIGH); // Считываем расстояние
    return distance;
}

// Функция открытия/закрытия сервопривода
void moveServo(ServoState state)
{

   int start= (state == OPEN) ? SERVO_OPEN_POSITION : SERVO_CLOSE_POSITION;
   int end= (state == OPEN) ? SERVO_CLOSE_POSITION : SERVO_OPEN_POSITION;
   
     for (int i = start; i <end;  state == OPEN ? i++ : i--)
    {
        servo.write(i); // Устанавливаем позицию сервопривода
        delay(10);      // Ждем между шагами
    }
}


// Функция для обработки препятствий
void takeBanka()
{

    // Если датчик обнаружил препятствие ближе, чем UZD_OBSTACLE_DISTANCE
    if (getForwardDistance() < UZD_OBSTACLE_DISTANCE)
    {
        drive(0, 0, STANDARD_DELAY * 2); // Останавливаем робота

        moveServo(CLOSE);

        drive(-baseSpeed, -baseSpeed, STANDARD_DELAY); // Двигаемся назад
        turn(LEFT);                                    // Поворачиваем влево
        // Пока левый датчик не обнаружит черную линию
        while (!isSensorOnBlack(analogRead(IR_SENSOR_LEFT_PIN)))
        {
            preg(); // Двигаемся по линии
        }
        drive(0, 0, STANDARD_DELAY * 2); // Останавливаем робота

        moveServo(OPEN);

        drive(-baseSpeed, -baseSpeed, STANDARD_DELAY * 2); // Двигаемся назад
        turn(LEFT);                                        // Поворачиваем влево
    }
}

// Функция для обработки перекрестка
void cross()
{
    drive(baseSpeed, baseSpeed, STANDARD_DELAY); // Двигаемся вперед
    drive(0, 0, STANDARD_DELAY);                 // Останавливаем робота
}

// Основной цикл программы
void loop()
{

    preg();
    if (isOnCross())
    {
        crossCount++;
        cross();
    };
    // if (crossCount==2){
    //     turn(LEFT);
    //     takeBanka();
    // };
}