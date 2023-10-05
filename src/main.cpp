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
const int UZ_F_SENSOR_PIN = 9;

// Устанавливаем скорость мотора
const int MOTOR_SPEED = 200;

// Устанавливаем порог для датчика линии
const int IR_SENSOR_THRESHOLD = 500;

// Устанавливаем коэффициент для ультразвукового датчика
const float UZDSENCE = 0.01723;

// Устанавливаем расстояния для датчика
const int UZD_OBSTACLE_DISTANCE = 6;
const int UZD_CROSS_DISTANCE = 30;

// Устанавливаем позиции сервопривода
const int SERVO_DOWN_POSITION = 30;
const int SERVO_UP_POSITION = 80;

// Устанавливаем стандартную задержку
const int STANDARD_DELAY = 500;

// Устанавливаем максимальную разрядность выхода на который подключены моторы
const int PIN_MAX_BIT = 255;

// Создание объекта для управления сервоприводом
Servo servo;

// Инициализация переменных
int V = 200;
float K = 1;
int min1 = 500, min2 = 500, max1 = 600, max2 = 600;
int step = 0;

// Определение направлений для функции поворота
enum Direction {
    LEFT,
    RIGHT
};

// Функция настройки начальных параметров
void setup() {
    servo.attach(SERVO_PIN); // Привязываем сервопривод к пину
    servo.write(SERVO_DOWN_POSITION); // Устанавливаем начальное положение сервопривода
    pinMode(MOTOR_L_DIRECTION_PIN, OUTPUT); // Устанавливаем режимы работы пинов
    pinMode(MOTOR_L_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_R_DIRECTION_PIN, OUTPUT);
    pinMode(MOTOR_R_SPEED_PIN, OUTPUT);
    pinMode(UZ_F_SENSOR_PIN, INPUT);
    pinMode(IR_SENSOR_LEFT_PIN, INPUT);
    pinMode(IR_SENSOR_RIGHT_PIN, INPUT);
    Serial.begin(9600); // Инициализация серийного порта для вывода отладочной информации
}

// Функция для управления двигателями
void move(int R, int L) {
    digitalWrite(MOTOR_L_DIRECTION_PIN, L > 0 ? HIGH : LOW); // Управляем направлением левого мотора
    analogWrite(MOTOR_L_SPEED_PIN, abs(L)); // Управляем скоростью левого мотора
    digitalWrite(MOTOR_R_DIRECTION_PIN, R > 0 ? HIGH : LOW); // Управляем направлением правого мотора
    analogWrite(MOTOR_R_SPEED_PIN, abs(R)); // Управляем скоростью правого мотора
}

// Функция для нормализации значений с датчиков
void normalize(int& d1, int& d2) {
    // Обновляем минимальные и максимальные значения
    min1 = min(min1, d1);
    min2 = min(min2, d2);
    max1 = max(max1, d1);
    max2 = max(max2, d2);
    // Преобразовываем значения датчиков
    d1 = map(analogRead(IR_SENSOR_LEFT_PIN), min1, max1, 0, 1000);
    d2 = map(analogRead(IR_SENSOR_RIGHT_PIN), min2, max2, 0, 1000);
}

// Проверяем, находится ли датчик на черной линии
bool isSensorOnBlack(int sensorValue) {
    return sensorValue < IR_SENSOR_THRESHOLD;
}

// Проверяем, находятся ли оба датчика на черной линии
bool isOnCross(int leftSensor, int rightSensor) {
    return isSensorOnBlack(leftSensor) && isSensorOnBlack(rightSensor);
}

// Функция для начального движения робота
void start() {
    move(MOTOR_SPEED, MOTOR_SPEED);                // Запускаем движение обоих моторов с базовой скоростью 
    delay(STANDARD_DELAY * 2); // Ждем удвоенное стандартное время задержки
    step++;                    // Увеличиваем значение шага на 1 для перехода к следующему этапу в логике управления
}
// Функция для поворота
void turn(Direction direction) {
    // Если направление влево
    if (direction == LEFT) {
        move(V, -V); // Поворачиваем влево
        delay(STANDARD_DELAY); // Ждем
        // Пока левый датчик не обнаружит черную линию
        while (!isSensorOnBlack(analogRead(IR_SENSOR_LEFT_PIN))) {
            move(-V, V);
        }
        // Пока правый датчик не обнаружит черную линию
        while (!isSensorOnBlack(analogRead(IR_SENSOR_RIGHT_PIN))) {
            move(-V, V);
        }
    } else { // Если направление вправо
        move(-V, V); // Поворачиваем вправо
        delay(STANDARD_DELAY); // Ждем
        // Пока правый датчик не обнаружит черную линию
        while (!isSensorOnBlack(analogRead(IR_SENSOR_RIGHT_PIN))) {
            move(V, -V);
        }
    }
}

void cross();

// Функция для управления движением по линии
void preg() {
    int d1 = analogRead(IR_SENSOR_LEFT_PIN); // Читаем значение левого датчика
    int d2 = analogRead(IR_SENSOR_RIGHT_PIN); // Читаем значение правого датчика
    normalize(d1, d2); // Нормализуем значения
    // Выводим значения датчиков в серийный порт для отладки
    Serial.print("Left Sensor: ");
    Serial.print(d1);
    Serial.print(", Right Sensor: ");
    Serial.println(d2);

    // Если робот на перекрестке, вызываем функцию cross()
    if (isOnCross(d1, d2)) {
        cross();
    } else { // Иначе регулируем направление движения
        int E = d1 - d2; //Ошибка- разность показаний датчиков
        int U = E * K; // Корректировка ошибки умножением на коэффициент
        int leftMotorSpeed = MOTOR_SPEED + U; //Корректируем скорость и направление левого мотора
        int rightMotorSpeed = MOTOR_SPEED- U; //Корректируем скорость и направление правого мотора
        leftMotorSpeed = constrain(leftMotorSpeed, -PIN_MAX_BIT, PIN_MAX_BIT); // Приводим значение скорости к диапазону +/- MOTOR_SPEED
        rightMotorSpeed = constrain(rightMotorSpeed, -PIN_MAX_BIT, PIN_MAX_BIT); // Приводим значение скорости к диапазону +/- MOTOR_SPEED
        move(leftMotorSpeed, rightMotorSpeed); // Вызываем функцию движения и передаем ей скорректированные значения скоростей мотров
    }
}

// Функция для считывания данных с ультразвукового датчика
int uzdF() {
    digitalWrite(UZ_F_SENSOR_PIN, LOW); // Отправляем сигнал LOW на пин
    delayMicroseconds(2); // Ждем немного
    digitalWrite(UZ_F_SENSOR_PIN, HIGH); // Отправляем сигнал HIGH на пин
    delayMicroseconds(10); // Ждем немного
    digitalWrite(UZ_F_SENSOR_PIN, LOW); // Отправляем сигнал LOW на пин
    int distance = UZDSENCE * pulseIn(UZ_F_SENSOR_PIN, HIGH); // Считываем расстояние
    // Выводим расстояние в серийный порт для отладки
    Serial.print("UZD Distance: ");
    Serial.println(distance);
    return distance;
}

// Функция для обработки препятствий
void banka() {
    // Если датчик обнаружил препятствие ближе, чем UZD_OBSTACLE_DISTANCE
    if (uzdF() < UZD_OBSTACLE_DISTANCE) {
        move(0, 0); // Останавливаем робота
        delay(STANDARD_DELAY * 2); // Ждем
        // Поднимаем сервопривод
        for (int i = SERVO_DOWN_POSITION; i < SERVO_UP_POSITION; i++) {
            servo.write(i); // Устанавливаем позицию сервопривода
            delay(10); // Ждем между шагами
        }
        move(-V, -V); // Двигаемся назад
        delay(STANDARD_DELAY); // Ждем
        turn(LEFT); // Поворачиваем влево
        // Пока левый датчик не обнаружит черную линию
        while (!isSensorOnBlack(analogRead(IR_SENSOR_LEFT_PIN))) {
            preg(); // Двигаемся по линии
        }
        move(0, 0); // Останавливаем робота
        delay(STANDARD_DELAY * 2); // Ждем
        // Опускаем сервопривод
        for (int i = SERVO_UP_POSITION; i > SERVO_DOWN_POSITION; i--) {
            servo.write(i); // Устанавливаем позицию сервопривода
            delay(10); // Ждем между шагами
        }
        move(-V, -V); // Двигаемся назад
        delay(STANDARD_DELAY * 2); // Ждем
        turn(LEFT); // Поворачиваем влево
        step++; // Увеличиваем значение шага
    } else {
        preg(); // Если препятствия нет, продолжаем двигаться по линии
    }
}

// Функция для обработки перекрестка
void cross() {
    move(V, V); // Двигаемся вперед
    delay(STANDARD_DELAY); // Ждем
    move(0, 0); // Останавливаем робота
    delay(STANDARD_DELAY); // Ждем
    turn(LEFT); // Поворачиваем влево
    move(0, 0); // Останавливаем робота
    delay(STANDARD_DELAY); // Ждем
    // Если датчик обнаружил препятствие на расстоянии меньше UZD_CROSS_DISTANCE
    if (uzdF() < UZD_CROSS_DISTANCE) {
        banka(); // Обрабатываем препятствие
    } else {
        turn(LEFT); // Иначе поворачиваем влево
        turn(LEFT); // И еще раз поворачиваем влево
        move(0, 0); // Останавливаем робота
    }
    step++; // Увеличиваем значение шага
}

// Основной цикл программы
void loop() {
    // В зависимости от текущего шага вызываем соответствующую функцию
    switch (step) {
        case 0:
            start(); // Начальное движение
            break;
        case 1:
            cross(); // Обработка перекрестка
            break;
        case 2:
            banka(); // Обработка препятствия
            break;
    }
}