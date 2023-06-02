#include <Arduino.h>

void readEncoderRaw(int encoderA, int encoderB)
{
    int a = digitalRead(encoderA);
    int b = digitalRead(encoderB);

    Serial.print(a * 5); // scale the values for easier visualization
    Serial.print(" ");
    Serial.print(b * 5);
}

// relies on interrupts and I can't pass variables to the attachInterrupt function at the moment.
// int readEncoder(int encoderPosition, int encoderB)
// {
//     int b = digitalRead(encoderB);

//     if (b > 0)
//     {
//         encoderPosition++;
//     }
//     else
//     {
//         encoderPosition--;
//     }

//     return encoderPosition;
// }

void getRPM()
{

    Serial.print(" ");
    Serial.println();
}

void getDistance()
{

    Serial.print(" ");
    Serial.println();
}