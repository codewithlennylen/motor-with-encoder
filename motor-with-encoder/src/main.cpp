#include <Arduino.h>
#include <basic_motor.h>

void setup()
{
  motor_setup();
}

void loop()
{
  go_forward();
  delay(3000);
  // stop_all();
  // delay(1000);
  // go_backwards();
  // delay(3000);
  // go_right();
  // delay(1000);
  // go_left();
  // delay(1000);
  stop_all();
  delay(1000);
}
