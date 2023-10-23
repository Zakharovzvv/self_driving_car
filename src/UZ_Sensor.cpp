#include <NewPing.h> // Подключение библиотеки для управления датчиком расстояния
#include <Header.h>
#include <Console.h>

NewPing sonar(9, 10); // NewPing setup of pins and maximum distance.

void uzdImpulse(int pin)
{
    digitalWrite(UZ_F_TRIGGER_PIN, LOW); // Сбрасываем датчик
    delayMicroseconds(2);                //

    digitalWrite(UZ_F_TRIGGER_PIN, HIGH); // Отправляем импульс до объекта
    delayMicroseconds(10);
    digitalWrite(UZ_F_TRIGGER_PIN, LOW);
}
// Функция для считывания данных с ультразвукового датчика
int getDistance()
{

    // long duration =pulseIn(UZ_F_ECHO_PIN, HIGH); //Получаем время прохождения импульса до объекта и обратно

    //    int distance = duration * SOUND_SPEED/2 ; // Считаем расстояние до объекта
    int distance = sonar.ping_cm(); // Считаем расстояние до объекта

#if !DEBUG
    console("d=", distance);
#endif

    return distance;
}