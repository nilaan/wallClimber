#include "DualMC33926MotorShield.h"

// Constructors ////////////////////////////////////////////////////////////////

DualMC33926MotorShield::DualMC33926MotorShield()
{
  //Pin map
  _nD2 = 34;
  _D1 = 38;

  _nD2_2 = 48;
  _D1_2 = 52; 

  _M1DIR = 12;
  _M2DIR = 7;

  _EN = 44;
}

// Public Methods //////////////////////////////////////////////////////////////
void DualMC33926MotorShield::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1DIR,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
  pinMode(_M2DIR,OUTPUT);
  pinMode(_M2PWM,OUTPUT);
  
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH); // default to on  
  pinMode(_D1,OUTPUT);
  digitalWrite(_D1,LOW); // default to off
  
  pinMode(_nD2_2,OUTPUT);
  digitalWrite(_nD2_2,HIGH); // default to on
  pinMode(_D1_2,OUTPUT);
  digitalWrite(_D1_2,LOW); // default to off

  pinMode(_EN,OUTPUT);
  digitalWrite(_EN,HIGH);


  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
  // Timer 1 configuration
  // prescaler: clockI/O / 1
  // outputs enabled
  // phase-correct PWM
  // top of 400
  //
  // PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  TCCR1A = 0b10100000;
  TCCR1B = 0b00010001;
  ICR1 = 400;
  #endif
}
// Set speed for motor 1, speed is a number betwenn -400 and 400
void DualMC33926MotorShield::setM1Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
  OCR1A = speed;
  #else
  analogWrite(_M1PWM,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  #endif
  if (reverse)
    digitalWrite(_M1DIR,HIGH);
  else
    digitalWrite(_M1DIR,LOW);
}

// Set speed for motor 2, speed is a number betwenn -400 and 400
void DualMC33926MotorShield::setM2Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
  OCR1B = speed;
  #else
  analogWrite(_M2PWM,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  #endif
  if (reverse)
    digitalWrite(_M2DIR,HIGH);
  else
    digitalWrite(_M2DIR,LOW);
}

// Set speed for motor 1 and 2
void DualMC33926MotorShield::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}
