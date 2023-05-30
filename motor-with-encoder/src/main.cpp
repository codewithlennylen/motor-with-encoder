#include <Arduino.h>
// #include <motor_encoder.h>
#include <basic_motor.h>

// Define pins connected the encoders A & B
// #define ENCA 2 // Yellow
// #define ENCB 3 // Green

// Motor A connections
// int enA = 10;
// int in1 = 8;
// int in2 = 7;
// int motorSpeed = 230;

// int pos = 0;
// long prevT = 0;
// float eprev = 0;
// float eintegral = 0;

// int encoderPosition = 0;

// void readEncoder();
// void setMotor(int dir, int pwmPin, int pwmValue, int in1, int in2);

void setup()
{

  // Serial.begin(9600); // Initialize the serial port
  // pinMode(ENCA, INPUT);
  // pinMode(ENCB, INPUT);
  // pinMode(enA, OUTPUT);
  // pinMode(in1, OUTPUT);
  // pinMode(in2, OUTPUT);

  motor_setup();
  stop_all();
  // attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop()
{
  go_forward();
  // delay(5000);
  // stop_all();
  // delay(200);
  // go_backwards();
  // delay(5000);
  // stop_all();
  // delay(2000);
  // Serial.println(encoderPosition);
  // setMotor(1, enA, motorSpeed, in1, in2);
  // delay(2000);
  // setMotor(0, enA, motorSpeed, in1, in2);
  // delay(1000);
  // // Serial.println(encoderPosition);
  // setMotor(-1, enA, motorSpeed, in1, in2);
  // delay(2000);
  // // Serial.println(encoderPosition);
  // setMotor(0, enA, motorSpeed, in1, in2);
  // delay(1000);
  // Serial.println(encoderPosition);

  // set target position
  // int target = 1200;
  // int target = 250*sin(prevT/1e6)

  // PID Constants
  // float kp = 1;
  // float ki = 0;
  // float kd = 0;

  // time difference
  // long currT = micros();

  // float deltaT = ((float)(currT - prevT)) / 1e6; // deltaT in seconds
  // prevT = currT;

  // // calculate error
  // // int e = target - encoderPosition;
  // int e = encoderPosition - target;

  // // derivative
  // float dedt = (e - eprev) / (deltaT);

  // // integral
  // eintegral = eintegral + e * deltaT;

  // // calculate control signal
  // float u = kp * e + ki * eintegral + kd * dedt;

  // // set motor power
  // float pwr = fabs(u);
  // if (pwr > 255)
  // {
  //   pwr = 255;
  // }

  // // set motor direction
  // int dir = 1;
  // if (u < 0)
  // {
  //   dir = -1;
  // }

  // // signal the motor
  // // setMotor(dir, enA, pwr, in1, in2);

  // // store previous error
  // eprev = e;

  // Serial.print("Target Position: ");
  // Serial.print(target);
  // Serial.print(" ");
  // Serial.print("Measured Position: ");
  // Serial.print(encoderPosition);
  // Serial.print(" ");
  // Serial.print("Error: ");
  // Serial.print(e);
  // Serial.print(" ");
  // Serial.print("Control Signal: ");
  // Serial.print(u);
  // Serial.print(" ");
  // Serial.println();
}
/*
void readEncoder()
{
  int b = digitalRead(ENCB);

  if (b > 0)
  {
    encoderPosition++;
  }
  else
  {
    encoderPosition--;
  }
}

void setMotor(int dir, int pwmPin, int pwmVal, int in1, int in2)
{
  analogWrite(pwmPin, pwmVal);

  if (dir == 1)
  {
    Serial.println("Forward");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1)
  {
    Serial.println("Backward");
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    Serial.println("Stop");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
*/