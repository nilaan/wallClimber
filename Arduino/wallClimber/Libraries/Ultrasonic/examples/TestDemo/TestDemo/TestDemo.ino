
// 2 Sonars4
#define ECHO_L 2
#define TRIGGER_L 4
#define ECHO_R 16
#define TRIGGER_R 18

int32_t duration, sonarR, sonarL; // Duration used to calculate distance


void setup() {
 Serial.begin (9600);
 pinMode(TRIGGER_R, OUTPUT);
 pinMode(ECHO_R, INPUT);
 pinMode(TRIGGER_L, OUTPUT);
 pinMode(ECHO_L, INPUT);
}

void loop() {
/* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
 digitalWrite(TRIGGER_R, LOW); 
 delayMicroseconds(2); 

 digitalWrite(TRIGGER_R, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(TRIGGER_R, LOW);
 duration = pulseIn(ECHO_R, HIGH);
 
 //Calculate the distance (in cm) based on the speed of sound.
 sonarR = duration/58.2;

 digitalWrite(TRIGGER_L, LOW); 
 delayMicroseconds(2); 

 digitalWrite(TRIGGER_L, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(TRIGGER_L, LOW);
 duration = pulseIn(ECHO_L, HIGH);
 
 //Calculate the distance (in cm) based on the speed of sound.
 sonarL = duration/58.2;
 
 Serial.print(sonarL);
 Serial.print("\t");
 Serial.println(sonarR);
 
 //Delay 50ms before next reading.
 delay(50);
}
