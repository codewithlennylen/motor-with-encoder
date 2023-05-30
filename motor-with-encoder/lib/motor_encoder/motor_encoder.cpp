#include <Arduino.h>

int encoderPosition = 0;

void readEncoderRaw(int encoderA, int encoderB)
{
    int a = digitalRead(encoderA);
    int b = digitalRead(encoderB);

    Serial.print(a * 5); // scale the values for easier visualization
    Serial.print(" ");
    Serial.print(b * 5);
    Serial.println();
}

int readEncoder(int encoderB)
{
    int b = digitalRead(encoderB);

    if (b > 0)
    {
        encoderPosition++;
    }
    else
    {
        encoderPosition--;
    }

    return encoderPosition;
}