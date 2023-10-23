#include <Arduino.h>
#include <Header.h>
#include <IR_Sensor.h>
#include <Drive.h>


// Проверяем, что оба датчика на черной линии. Значит мы на перекрестке
bool isOnCros()
{
    return isSensorOnBlack(getIRSensorValue(IR_SENSOR_L_PIN)) && isSensorOnBlack(getIRSensorValue(IR_SENSOR_R_PIN));
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