#define DEBUG 0

#if DEBUG
#include "avr8-stub.h"
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

// Устанавливаем порог для датчика линии
const int IR_SENSOR_THRESHOLD = 500;

const float SOUND_SPEED = 0.034; // Скорость звука см/сек

// Устанавливаем расстояния для датчика
const int UZD_OBSTACLE_DISTANCE = 6;
const int UZD_CROSS_DISTANCE = 30;

// Устанавливаем позиции сервопривода
const int SERVO_OPEN_POSITION = 30;
const int SERVO_CLOSE_POSITION = 80;

// Устанавливаем стандартную задержку
const int STANDARD_DELAY = 500;

const int PIN_MAX_BIT = 250;     // Устанавливаем максимальную разрядность выхода на который подключены моторы
const int UZ_MAX_DISTANCE = 200; // Устанавливаем максимальное расстояние до объекта

const int LINES_FOR_TUNE = 2; // Количество чкрных линий, которые надо увидеть датчику при повороте машины

// Создание объекта для управления сервоприводом
Servo servo;
NewPing sonar(9, 10); // NewPing setup of pins and maximum distance.

// Инициализация переменных
int baseSpeed = 60; // Базовая скорость моторов
float pregK = 0.3;  // Коэффициент чувствительности П регулятора
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

#if !DEBUG
// Первый шаблон для завершения рекурсии
template <typename T>
void consoleInternal(const char *name, T value)
{
    Serial.print(name);
    Serial.print(": ");
    Serial.println(value);
}

// Рекурсивный шаблон для обработки переменного количества аргументов
template <typename T, typename... Args>
void consoleInternal(const char *name, T value, Args... args)
{
    Serial.print(name);
    Serial.print(": ");
    Serial.print(value);
    Serial.print(", ");
    consoleInternal(args...);
}

// Функция для вывода отладочной информации
template <typename... Args>
void console(Args... args)
{
    consoleInternal(args...);
    Serial.println(); // добавляем новую строку после вывода
}
#endif
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
    servo.write(SERVO_OPEN_POSITION); // Устанавливаем начальное положение сервопривода

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
    return sensorValue < IR_SENSOR_THRESHOLD;
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
    //    console("Left Sensor: ", d1, "Right Sensor: ", d2);
}

// Функция для обработки перекрестка
void driveCross()
{
    drive(0, 0, 500);
    while (isOnCros())
    {
        drive(baseSpeed, baseSpeed);
    }

    // drive(baseSpeed, baseSpeed, STANDARD_DELAY); // Двигаемся вперед
    // drive(0, 0, STANDARD_DELAY);                 // Останавливаем робота
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
        for (int i = SERVO_OPEN_POSITION; i < SERVO_CLOSE_POSITION; i++)
        {
            servo.write(i); // Устанавливаем позицию сервопривода
            delay(10);      // Ждем между шагами
#if !DEBUG
            console("servo=", i);
#endif
        }
    }
    else
    {
        for (int i = SERVO_CLOSE_POSITION; i > SERVO_OPEN_POSITION; i--)
        {
            servo.write(i); // Устанавливаем позицию сервопривода
            delay(10);      // Ждем между шагами
#if !DEBUG
            console("servo=", i);
#endif
        }
    }
}

// Функция для поворота
void turn(Direction direction)
{
    int linesCrossed = 0; // Переменная для подсчета количества линий, которые пересек датчик
    bool flag = true;     // Флаг который переключается каждый раз, когда мв пересекаем границу черного/белого

    do
    {
        drive(baseSpeed * direction == LEFT ? -1 : 1, baseSpeed * direction == LEFT ? 1 : -1);
        bool iROnBlack = isSensorOnBlack(getIRSensorValue(direction == LEFT ? IR_SENSOR_R_PIN : IR_SENSOR_L_PIN));
        if (iROnBlack && flag)
        {
            linesCrossed++;
            flag = !flag;
        }
        else if (!iROnBlack)
        {
            flag = !flag;
        };

    } while (linesCrossed < LINES_FOR_TUNE);
    drive(0, 0);
}

// Функция для обработки препятствий
void takeBanka()
{

    // Едем пока не приблизимся к банке на расстояние захвата
    while (getForwardDistance() > UZD_OBSTACLE_DISTANCE)
    {
        preg();
    };

    drive(0, 0, STANDARD_DELAY); // Останавливаем робота

    moveServo(CLOSE); //Берем банку

    drive(-baseSpeed, -baseSpeed, STANDARD_DELAY); // Двигаемся назад
    turn(LEFT);                                    // Поворачиваем влево
}

// Основной цикл программы
void loop()
{
    // turn(LEFT);
    // while (true)
    // {
    //     /* code */
    // }

     preg(); //Едем по линии

    // Обрабатываем перекрестки
    //  if (isOnCros())
    //  {
    //      crossCount++;
    //      driveCross();
    //  };
    //Если увидели банку - берем ее
    if (getForwardDistance() < UZD_OBSTACLE_DISTANCE*1.5){
        takeBanka();
    }
}

void check()
{
    drive(baseSpeed, baseSpeed, 500);
    drive(0, 0, 0);
    getForwardDistance();
    moveServo(CLOSE);
    moveServo(OPEN);
}