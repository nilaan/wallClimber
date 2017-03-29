#include "DualMC33926MotorShield.h"
#include <MPU6050.h>


#define OUTPUT_READABLE_ACCELGYRO

/* encoder not being used
#define ENC_L1 2
#define ENC_L2 3
#define ENC_R1 18
#define ENC_R2 19
*/

/* Already defined in default constructor
  _nD2_2 = 48;
  _D1_2 = 52; 

  _M1DIR = 12;
  _M2DIR = 7;

  _EN = 44;

  _M1PWM = 10;
  _M2PWM = 9;
*/

// 2 Sonars
#define ECHO_L 2
#define TRIGGER_L 4
#define ECHO_R 16
#define TRIGGER_R 18


DualMC33926MotorShield md;
int16_t ax, ay, az;
int32_t ax_t, ay_t, az_t;
uint16_t tofDistance;
int32_t sonarL;
int32_t sonarR;
int32_t duration;
int32_t wallRefR = 0;
int32_t wallRefL = 0;
int32_t prevSonarL = 0;
int32_t prevSonarR = 0;
uint16_t s1 = 0, s2 = 0;
uint16_t state = 0;
uint16_t i = 0;
uint16_t gyro_count = 200;
uint32_t startTime;
int32_t counter = 0;
float exittime = 0;
float entrytime = 0;
Vector norm;
uint16_t interruptCounter = 0;
uint16_t flag = 0;
uint8_t turned = 0;

// for new mpu
MPU6050 mpu;

// Timers
unsigned long timer = 0;

float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
int gyro_counter = 0;

void errorHandle(uint8_t error)
{
  Serial.print("Error Code: ");
  Serial.print(error);
  Serial.println();
  digitalWrite(LED_BUILTIN, HIGH); 
}


void updateSensors()
{
  /*
  for (i = 0; i < gyro_count; i++)
  {
    gyroSensor.getAcceleration(&ax, &ay, &az);
    ax_t += ax;
    ay_t += ay;
    az_t += az;
  }

  ax_t /= gyro_count;
  ay_t /= gyro_count;
  az_t /= gyro_count;
/*  
  distance = tofSensor.readRangeSingleMillimeters();
  if (tofSensor.timeoutOccurred())
    errorHandle(2);
*/
  for (i = 0; i < gyro_count; i++)
  {
    Vector rawAccel = mpu.readRawAccel();
    ax_t += rawAccel.XAxis;
    ay_t += rawAccel.YAxis;
    az_t += rawAccel.ZAxis;
  }
  ax_t /= gyro_count;
  ay_t /= gyro_count;
  az_t /= gyro_count;
  
}

void updateProximity()
{
/*
  duration = 0;
  while (duration < 582 || duration > 23280)
  {
    digitalWrite(TRIGGER_R, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_R, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_R, LOW);
    duration = pulseIn(ECHO_R, HIGH);
  }
  sonarR = duration/58.2;
  */
  duration = 0;
  while (duration < 582 || duration > 23280)
  {
    digitalWrite(TRIGGER_L, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_L, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_L, LOW);
    duration = pulseIn(ECHO_L, HIGH);
  }
  sonarL = duration/58.2;
/*
  Serial.print(wallRefL);
  Serial.print("\t");
  Serial.print(wallRefR);
  Serial.print("\t");
  Serial.print(sonarL);
  Serial.print("\t");
  Serial.print(sonarR); 
  Serial.print("\t");
  Serial.print(wallRefL-sonarL);
  Serial.print("\t");
  Serial.println(wallRefR-sonarR);
*/
}

void motor()
{
  md.setSpeeds(s1, s2);
}

void turnLeft()
{
  s1 = -21;
  s2 = 360;
  motor();
  pitch = roll = yaw = 0;
  while (abs(yaw - 90) > 5)
  {
    timer = millis();
    norm = mpu.readNormalizeGyro();
    pitch = pitch + norm.YAxis * timeStep;
    roll = roll + norm.XAxis * timeStep;
    yaw = yaw + norm.ZAxis * timeStep;
    Serial.println(yaw);

    delay((timeStep*1000) - (millis()-timer));    
  }
  s1 = s2 = 0;
  motor();
}

void turnLeftNoGyro()
{
  s1 = -21;
  s2 = 0;
  motor();
  delay(400);
  s2 = 390; 
  motor();
  delay(600);
  s1 = s2 = 0;
  motor();
}


void turn180()
{
  s1 = -21;
  s2 = 360;
  motor();
  while (abs(yaw - 180) > 5)
  {
    timer = millis();
    norm = mpu.readNormalizeGyro();
    pitch = pitch + norm.YAxis * timeStep;
    roll = roll + norm.XAxis * timeStep;
    yaw = yaw + norm.ZAxis * timeStep;
    Serial.println(yaw);

    delay((timeStep*1000) - (millis()-timer));    
  }

  s1 = 360;
  s2 = 280;
  motor();
  delay (500);
  s1 = s2 = 0;
  motor();
  
}

void turn180NoGyro()
{
  s1 = -21;
  s2 = 360;
  motor();
  delay(1200);
  s1 = s2 = 0;
  motor();
  
}

void turnRight()
{
  s1 = 360;
  s2 = -21;
  motor();
  pitch = roll = yaw = 0;
  while (abs(yaw + 90) > 5)
  {
    timer = millis();
    norm = mpu.readNormalizeGyro();
    pitch = pitch + norm.YAxis * timeStep;
    roll = roll + norm.XAxis * timeStep;
    yaw = yaw + norm.ZAxis * timeStep;
    Serial.println(yaw);

    delay((timeStep*1000) - (millis()-timer));    
  }
  s1 = s2 = 0;
  motor();
}

void driveStraight(int32_t &type, int16_t tol)
{
  s1 = 350;
  s2 = 350;
  motor();

  pitch = roll = yaw = 0;
  while (type < tol)
  {
    timer = millis();
    norm = mpu.readNormalizeGyro();
    pitch = pitch + norm.YAxis * timeStep;
    roll = roll + norm.XAxis * timeStep;
    yaw = yaw + norm.ZAxis * timeStep;
    if (yaw > 0)
    {
      s2 += 30;
      s1 -= 30;
      motor();
    }
    else if (yaw < 0)
    {
      s1 += 30;
      s2 -= 30;
      motor();
    }
    updateSensors;
    delay((timeStep*1000) - (millis()-timer));
  }
}

void setup() 
{
    Serial.begin(115200);
    //Wire.begin();
  
    pinMode(TRIGGER_L, OUTPUT);
    pinMode(ECHO_L, INPUT);
  
    pinMode(TRIGGER_R, OUTPUT);
    pinMode(ECHO_R, INPUT);  
  
    /*
  
    gyroSensor.initialize();
    if (!gyroSensor.testConnection())
      errorHandle(1);
      */
     md.init();
    
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
      Serial.println("BROOOO Could not find a valid MPU6050 sensor, check wiring!");
      delay(500);
    }
  
    mpu.calibrateGyro();
    mpu.setThreshold(3);
  
  /*
    tofSensor.init();
    tofSensor.setTimeout(500);
    tofSensor.startContinuous();
  */
}

void loop() {

  s1 = 400;
  s2 = 380;
  motor();
/*
  //md.setSpeeds(100, 0);
  //delay(50);
  md.setSpeeds(100, 100);
  delay(50);
  motor();
*/
  state = 0;

  while (state < 3)
  {
    updateSensors();
   
    if (state == 0)
    {
      // detect get onto wall
      
      if (ay_t > 15000)
      {
        s1 = 390;
        motor();
        state++;
      }
    }
    
    if (state == 1)
    {
      // detect get over edge 1
      if (ay_t < 13500)
      {
        s1 = 200;
        s2 = 192;
        motor();
      }
    
      // detect get over edge 2
      if (ay_t < -7500)
      {
        s1 = 30;
        s2 = 26;
        motor();
      }
   
      // detect get over edge 3
      if (ay_t < -16000)
      {
        s1 = 400;
        s2 = 370;
        motor();
        state++;
      }
    }

    if (state == 2)
    {
      // detect get back on floor
      if (az_t > 15500)
      {
        state++;
//        s1 = s2 = -200;
//        motor();
//        delay(1000);
//        s1 = s2 = 200;
//        motor();
//        delay(500);
//        s1 = s2 = 0;
//        motor();
//        delay(50);
//        turnRight();
//        s1 = s2 = -200;
//        motor();
        s1 = s2 = 350;
        motor();
        startTime = millis();
        delay(800);
        updateProximity();
      }
    }
/*
    Serial.print(state);
    Serial.print("\t");
    Serial.print(ax_t);
    Serial.print("\t");
    Serial.print(ay_t);
    Serial.print("\t");
    Serial.println(az_t);
*/
  }
  
  i = 0;
  while (i < 6)
  {
    i++;
    updateProximity();
  }
    
  i = 0;

  // enter pole detection
  while(state == 3)
  {
    wallRefL += sonarL;
    counter++;
    
    updateProximity();
    
    if ((wallRefL/counter - sonarL) > 25)
    {
      i = 1;
      s1 = s2 = -300;
      motor();
      delay(250);
      state++;
      s1 = s2 = 0;
      motor();
      turnLeftNoGyro();
    }
//    if ((millis()-startTime) > 2000 && s1 == -200)
    if ((millis()-startTime) > 4500 && turned == 0)
    {
      turn180();
      s1 = s2 = 300;
      motor();
      turned = 1;
    }

    Serial.print(wallRefL);
    Serial.print("\t");
    Serial.println(sonarL);
  }
  Serial.println("Detected Pole.");

  s1 = s2 = 300;
  motor();
  updateSensors();
  while (az_t > 14500)
  {
    updateSensors();
  }
  s1 = 0;
  s2 = 0;
  motor();
  
  while(1);
}
