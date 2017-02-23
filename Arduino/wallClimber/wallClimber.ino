#include <Encoder.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "VL53L0X.h"
#include "DualMC33926MotorShield.h"

#define OUTPUT_BINARY_ACCELGYRO

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY

#define ENC_L1 2
#define ENC_L2 3
#define ENC_R1 18
#define ENC_R2 19

/* Already defined in default constructor
#define M1DIR 7
#define M1PWM 9
#define M1FB A0
#define M2DIR 8
#define M2PWM 10
#define M2FB A1
#define nD2 4
#define nSF 12
*/

VL53L0X tofSensor;
MPU6050 gyroSensor;
Encoder encoderLeft(ENC_L1, ENC_L2);
Encoder encoderRight(ENC_R1, ENC_R2);

DualMC33926MotorShield md;
int16_t ax, ay, az;
int16_t gx, gy, gz;
uint16_t distance;
uint16_t s1, s2;

void errorHandle(uint8_t error)
{
  Serial.print("Error Code: ");
  Serial.print(error);
  Serial.println();
}

void updateSensors()
{
  gyroSensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  distance = tofSensor.readRangeSingleMillimeters();
  if (tofSensor.timeoutOccurred())
    errorHandle(2);
}

// Range from -400 to 400
void motor()
{
  md.setSpeeds(s1, s2);
  if (md.getFault())
    errorHandle(3);
}

bool state1()
{
  bool success = 0;

  return success;
}

bool state2()
{
  bool success = 0;

  return success;
}
bool state3()
{
  bool success = 0;

  return success;
}

void setup() 
{
  Serial.begin(9600);
  Wire.begin();

  tofSensor.init();
  tofSensor.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif

  gyroSensor.initialize();
  if (!gyroSensor.testConnection())
    errorHandle(1);

  md.init();
}

void loop() {
  // put your main code here, to run repeatedly:

  bool stateFlag1 = 0;
  bool stateFlag2 = 0;
  bool stateFlag3 = 0;

  while (stateFlag1 == 0)
    state1();
  while (stateFlag2 == 0)
    state2();
  while (stateFlag3 == 0)
    state3();
  while(1);

}
