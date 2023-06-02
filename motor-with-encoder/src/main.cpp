#include <Arduino.h>
#include <motor_encoder.h>
// #include <motor_distance.h>

// Define pins connected the encoders A & B
#define RIGHT_ENCA 18 // Yellow
#define RIGHT_ENCB 19 // Green
#define LEFT_ENCA 2   // Yellow
#define LEFT_ENCB 3   // Green

// Motor A connections
int enA = 4;
int in1 = 8;
int in2 = 9;

// Motor B connections
int enB = 5;
int in3 = 10;
int in4 = 11;

// More motor variables
int motorSpeed = 200;
int leftEncoderPosition = 0;
int rightEncoderPosition = 0;

// PID-related variables
long prevT = 0;
float eprev = 0;
float eintegral = 0;

// Function prototypes
void readLeftEncoder();
void readRightEncoder();
void setMotor(int dir, int pwmPin, int pwmValue, int in1, int in2);

void setup()
{

  // Initialize the serial port
  Serial.begin(9600);

  // Initialize motor pins
  pinMode(LEFT_ENCA, INPUT);
  pinMode(LEFT_ENCB, INPUT);
  pinMode(RIGHT_ENCA, INPUT);
  pinMode(RIGHT_ENCB, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCA), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA), readRightEncoder, RISING);
}

void loop()
{

  Serial.print("Left Encoder Position: ");
  Serial.print(leftEncoderPosition);
  Serial.print(" ");
  Serial.print("Right Encoder Position: ");
  Serial.print(rightEncoderPosition);
  Serial.print(" ");
  Serial.println();

  // Serial.print("Left Motor Encoder: ");
  // readEncoderRaw(LEFT_ENCA, LEFT_ENCB);
  // Serial.print(" Right Motor Encoder: ");
  // readEncoderRaw(RIGHT_ENCA, RIGHT_ENCB);
  // Serial.println();

  // ------------ MOTOR CONTROLLER (PID Loop) ----------------------

  // // set target position
  // int target = 1200;
  // // int target = 250*sin(prevT/1e6)

  // // PID Constants
  // float kp = 1;
  // float ki = 0;
  // float kd = 0.025;

  // // compute time difference
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

  // // signal the motor (Drive the motor)
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

  // int b = digitalRead(ENCB);

  //   if (b > 0)
  //   {
  //       encoderPosition++;
  //   }
  //   else
  //   {
  //       encoderPosition--;
  //   }

  // float distance = (encoderPosition / 65) * 0.065 * PI;

  // Serial.print(distance * 1000); // distance in mm
  // Serial.print(" ");
  // Serial.println();
  // getDistance();
}

void readLeftEncoder()
{
  int b = digitalRead(LEFT_ENCB);

  //? WARNING: left motor is inverted (mirror image of right motor)
  if (b > 0)
  {
    leftEncoderPosition--;
  }
  else
  {
    leftEncoderPosition++;
  }
}

void readRightEncoder()
{
  int b = digitalRead(RIGHT_ENCB);

  if (b > 0)
  {
    rightEncoderPosition++;
  }
  else
  {
    rightEncoderPosition--;
  }
}

void setMotor(int dir, int pwmPin, int pwmVal, int in1, int in2)
{
  analogWrite(pwmPin, pwmVal);

  if (dir == 1)
  {
    // Serial.println("Forward");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1)
  {
    // Serial.println("Backward");
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    // Serial.println("Stop");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
