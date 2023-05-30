#include <Arduino.h>
#include <motor_encoder.h>

// Define pins connected the encoders A & B
#define ENCA 2 // Yellow
#define ENCB 3 // Green

// Motor A connections
int enA = 8;
int in1 = 9;
int in2 = 10;
int motorSpeed = 100;

// PID-related variables
int encoderPosition = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void readEncoder();
void setMotor(int dir, int pwmPin, int pwmValue, int in1, int in2);

void setup()
{

  // Initialize the serial port
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop()
{

  setMotor(1, enA, motorSpeed, in1, in2);
  delay(3000);
  setMotor(0, enA, motorSpeed, in1, in2);
  delay(2000);
  setMotor(-1, enA, motorSpeed, in1, in2);
  delay(3000);
  setMotor(0, enA, motorSpeed, in1, in2);
  delay(2000);

  // ------------ MOTOR CONTROLLER (PID) ----------------------

  // set target position
  // int target = 1200;
  // int target = 250*sin(prevT/1e6)

  // PID Constants
  // float kp = 1;
  // float ki = 0;
  // float kd = 0;

  Serial.print(encoderPosition);
  Serial.print(" ");
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
  Serial.print("Measured Position: ");
  Serial.println(encoderPosition);
  // Serial.print(" ");
  // Serial.print("Error: ");
  // Serial.print(e);
  // Serial.print(" ");
  // Serial.print("Control Signal: ");
  // Serial.print(u);
  // Serial.print(" ");
  // Serial.println();
}

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
