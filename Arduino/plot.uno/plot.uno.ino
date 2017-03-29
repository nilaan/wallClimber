#define ECHO_L 2
#define TRIGGER_L 4
#define ECHO_R 16
#define TRIGGER_R 18
#include "DualMC33926MotorShield.h"

int32_t sonarL;
int32_t sonarR;
int32_t duration;
int count = 0;

void updateProximity()
{
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
}

  DualMC33926MotorShield md;
void setup() {
  Serial.begin(9600);

  pinMode(TRIGGER_L, OUTPUT);
  pinMode(ECHO_L, INPUT);

  pinMode(TRIGGER_R, OUTPUT);
  pinMode(ECHO_R, INPUT);  
  // put your setup code here, to run once:
  md.init();
  md.setSpeeds(300, 300);
}

void loop() {
  if (count < 1000)
  {
    updateProximity();
    // put your main code here, to run repeatedly:
    Serial.print(sonarL);
    Serial.print(" ");
    Serial.println(sonarR);
    count++;
  }
  else
  {
    Serial.println("done");
    md.setSpeeds(0, 0);
    while(1);
  }
}
