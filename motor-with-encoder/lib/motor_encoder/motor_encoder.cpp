#include <Arduino.h>

// int encoderPosition = 0;

struct MyTuple
{
    int value1;
    int value2;
};

// Foo SomeFunction()
// {
//     Foo result = { 5, 4 };
//     return result;
// }

void readEncoderRaw(int encoderA, int encoderB)
{
    int a = digitalRead(encoderA);
    int b = digitalRead(encoderB);

    // MyTuple result = { a, b };
    // return result;

    Serial.print(a * 5); // scale the values for easier visualization
    Serial.print(" ");
    Serial.print(b * 5);
    // Serial.println();
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