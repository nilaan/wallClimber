  #include "DualMC33926MotorShield.h"
  
  DualMC33926MotorShield md;
  
void setup()
{
  Serial.begin(115200);
  Serial.println("Dual MC33926 Motor Shield");
  md.init();
}

void loop()
{
  uint16_t s1 = 300;
  uint16_t s2 = 300;
  md.setSpeeds(s1, 0);
  delay(200);
  md.setSpeeds(s1, s2);
  delay(2000);
  md.setSpeeds(0, 0);
  /*
  for (int i = 0; i <= 400; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    delay(2);
  }
  delay(4000);
  for (int i = 400; i >= -1; i--)
  {
    md.setM1Speed(i);
    stopIfFault();
    delay(2);
  }
  delay(4000);  
  for (int i = -1; i <= 0; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    delay(2);
  }
*/

/*
  for (int i = 0; i <= 400; i++)
  {
    md.setM2Speed(i);
    stopIfFault2();
    delay(2);
  }
  
  for (int i = 400; i >= 1; i--)
  {
    md.setM2Speed(i);
    stopIfFault2();
    delay(2);
  }
*/
  
  while(1);
}
