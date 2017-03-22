#ifndef DualMC33926MotorShield_h
#define DualMC33926MotorShield_h

#include <Arduino.h>

class DualMC33926MotorShield
{
  public:  
    // CONSTRUCTORS
    DualMC33926MotorShield(); // Default pin selection.
    // PUBLIC METHODS
    void init(); // Initialize TIMER 1, set the PWM to 20kHZ. 
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
    unsigned int getM1CurrentMilliamps(); // Get current reading for M1. 
    unsigned int getM2CurrentMilliamps(); // Get current reading for M2.
    
  private:
    unsigned char _nD2;
    unsigned char _nD2_2;
    unsigned char _M1DIR;
    unsigned char _M2DIR;
    unsigned char _D1;
    unsigned char _D1_2;
    unsigned char _EN;
    static const unsigned char _M1PWM = 10;
    static const unsigned char _M2PWM = 9;

};

#endif