#include <Arduino.h>
#include <Header.h>
#include <Servo.h>   // Подключение библиотеки для управления сервоприводами
#include <UZ_Sensor.h> 
#include <IR_Sensor.h>  
Servo servo;

void initServo(){
        servo.attach(SERVO_PIN);          // Привязываем сервопривод к пину
    servo.write(servoOpenPosition); // Устанавливаем начальное положение сервопривода
}
// Функция открытия/закрытия сервопривода
void moveServo(ServoState state)
{

    if (state == OPEN)
    {
        for (int i = servoOpenPosition; i < servoClosePosition; i++)
        {
       //     servo.write(i); // Устанавливаем позицию сервопривода
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
    //        servo.write(i); // Устанавливаем позицию сервопривода
            delay(10);      // Ждем между шагами
#if !DEBUG
 //           console("servo=", i);
#endif
        }
    }
}

