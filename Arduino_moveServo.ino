#include <Servo.h>  // include the Servo library

//Objects for Controlling servo motor
Servo servo1;
Servo servo2;
Servo servo3;

float angleX = 40;
float angleY = 40;
float switchprev1 = 40;
float switchprev2 = 40;
float switchcurr1 = 40;
float switchcurr2 = 40;
int blinkrand = 0;
unsigned long mytime1 = millis();
bool flag = true;

void setup()
{
  Serial.begin(9600);

  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
}

void loop()
{
  if (Serial.available() > 1) //to check if atleast 2 bits are there
  {
    //Read from serial
    angleX = Serial.parseFloat();
    angleY = Serial.parseFloat();
  }
  if (flag == true)
  {
    blinkrand = random(1, 5000);
//    Serial.print("Generate Random Number: ");
//    Serial.println(blinkrand);
  }
  switchcurr1 = angleX * 0.05 + switchprev1 * 0.95;
  switchcurr2 = angleY * 0.05 + switchprev2 * 0.95;

  servo1.write(switchcurr1);
  servo2.write(switchcurr2);

  if (0 < blinkrand && blinkrand <= 20)
  {
    if (flag == true)
    {
      servo3.write(200);
//      Serial.println("Close eyelid");
      flag = false;
      mytime1 = millis();
    }
    if (millis() - mytime1 >= 500 && flag == false)
    {
      //    delay(500);
//      Serial.println("Open eyelid");
      mytime1 = millis();
      servo3.write(70);
      blinkrand = 100;
      flag = true;
    }
  }

  switchprev1 = switchcurr1;
  switchprev2 = switchcurr2;
  delay(10);
}
