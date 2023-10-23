#include <Arduino.h>
#include <Header.h>

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