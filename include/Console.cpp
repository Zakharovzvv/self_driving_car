#include <Arduino.h>
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