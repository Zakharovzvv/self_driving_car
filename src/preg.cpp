#include <Arduino.h>
#include <Header.h>
#include <Drive.h>
#include <Console.h>
#include <IR_Sensor.h>

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
