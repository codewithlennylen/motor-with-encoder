#include <Arduino.h>


void readEncoderRaw(int encoderA, int encoderB)
{
    int a = digitalRead(encoderA);
    int b = digitalRead(encoderB);


    Serial.print(a * 5); // scale the values for easier visualization
    Serial.print(" ");
    Serial.print(b * 5);
}

int readEncoder(int encoderPosition, int encoderB)
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

void getDistance(int encoderPosition, int encoderB, int encoderResolution, float wheelDiameter)
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

    float distance = (encoderPosition / encoderResolution) * wheelDiameter * PI;

    Serial.print(distance);
    Serial.print(" ");
    Serial.println();
}