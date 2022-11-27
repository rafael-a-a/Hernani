#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#include "SparkFunLIS3DH.h"
#include "math.h"
//-------------------------------------------------------------------------------------------------------//
#define _PWM_LOGLEVEL_        3
#include "RP2040_PWM.h"

//#define pin0    25    // PWM channel 4B, BUILTIN_LED
#define servo1    0   // PWM channel 0A
#define servo2    1     // PWM channel 1A
#define servo5    2     // PWM channel 2A
#define servo6    3     // PWM channel 3A
#define servo7    6     // PWM channel 0A
#define servo8    7    // PWM channel 5A
#define servo9    8    // PWM channel 6A
#define servo10   9    // PWM channel 7A
#define echoPin   16
#define trigPin   17


uint32_t PWM_Pins[]       = { servo1, servo2, servo5, servo6, servo7, servo8, servo9, servo10 };

#define NUM_OF_PINS       ( sizeof(PWM_Pins) / sizeof(uint32_t) )

typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

/*
// Input variables
uint8_t S1, prevS1, RE_S1;
uint8_t S2, prevS2, RE_S2;*/
long sonarDuration;
long distance;
/*
// Output variables

// Our finite state machines
fsm_t fsm1, fsm2, fsm3, fsm4, fsm5, fsm6, fsm7, fsm8, fsm9, fsm10, fsm11;
*/

unsigned long interval, last_cycle = 0;
unsigned long loop_micros;

void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

float dutyCycle[NUM_OF_PINS];

//creates pwm instance
RP2040_PWM* PWM_Instance[NUM_OF_PINS];
//creates accel instance
LIS3DH myIMU;

// calculates pwm necessary for each different servo
float calcDutyCycle(int servo, float angle){
  float dc;

  switch(servo){

    case servo1:
      dc = 10.0/180.0*angle + 2.8;
      return dc;
    case servo2:
      dc = 10.0/180.0*angle + 2.6;
      return dc;
    case servo5:
      dc = 10.0/180.0*angle + 2.95;
      return dc;
    case servo6:
      dc = 9.8/180.0*angle + 3;
      return dc;
    case servo7:
      dc = 10.0/180.0*angle + 2.45;
      return dc;
    case servo8:
      angle = 90 - angle;
      dc = 9.8/180.0*angle + 3;
      return dc;
    case servo9:
      angle = 90 - angle;
      dc = 9.7/180.0*angle + 2.8;
      return dc;
    case servo10:
      dc = 10.1/180.0*angle + 2.8;
      return dc;

  }

  return 3; // aprox 0º
}

float getDistance(){

  //returns distance from nearest object in cm
  
  int state=0;
  long auxtemp=micros();
  float distance=0;

  digitalWrite(trigPin,LOW);
  while (distance == 0){

    if(state == 0 && micros()-auxtemp > 2){
      state = 1;
      digitalWrite(trigPin,HIGH);
      auxtemp=micros();
      

    } else if (state == 1 && micros()-auxtemp > 10){
      digitalWrite(trigPin,LOW);
      sonarDuration = pulseIn(echoPin, HIGH);
      distance = sonarDuration * 0.034 / 2;
      auxtemp = micros();
      state = 0;
    }
    

  }
  return distance;

}

double getPitch(){

    //returns pitch in degrees

    double angle;
    double fz,fx,fy;

    fx = myIMU.readFloatAccelX();
    fz = myIMU.readFloatAccelZ();
    fy = myIMU.readFloatAccelY();
    angle = asin(fx/sqrt(pow(fx,2) + pow(fy,2) + pow(fz,2) ) ) * 180.0 / 3.14;

    return angle;

}

double getRoll(){

    //returns pitch in degrees

    double angle;
    double fz,fx,fy;

    fx = myIMU.readFloatAccelX();
    fz = myIMU.readFloatAccelZ();
    fy = myIMU.readFloatAccelY();
    angle = acos(fy/sqrt(pow(fx,2) + pow(fy,2) + pow(fz,2) ) ) * 180.0 / 3.14;

    return angle;

}

float frequency = 50;

double thetaP, thetaR;


void setup()
{
  //Accel SCL-GPIO5
  //Accel SDA-GPIO4
  //Accel VCC-3.3V
 
  interval = 10;
  //set_state(fsm1, 0);

  //uint32_t PWM_Pins[]       = { servo1, servo2, servo5, servo6, servo7, servo8, servo9, servo10 };

  pinMode(27,OUTPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);

  for (uint8_t index = 0; index < 8; index++)
  {
    PWM_Instance[index] = new RP2040_PWM(PWM_Pins[index], frequency, dutyCycle[index]);
  }

  myIMU.begin();
}


void loop()
{
  unsigned long now = millis();
  

  if(now - last_cycle > interval){

    //update inputs
    //distance = getDistance();
    thetaP = getPitch();
    thetaR = getRoll();

    Serial.println("Pitch:");
    Serial.println(thetaP, 2);
    Serial.println(thetaR,2);
    
    Serial.println(distance);

    //uint32_t PWM_Pins[]       = { servo1, servo2, servo5, servo6, servo7, servo8, servo9, servo10 };

    //servo testing
    dutyCycle[0] = calcDutyCycle(servo1,90);
    dutyCycle[1] = calcDutyCycle(servo2,90);
    dutyCycle[6] = calcDutyCycle(servo9,90);
    dutyCycle[5] = calcDutyCycle(servo8,90);
    PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
    PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);


    delay(1000);
    
    dutyCycle[0] = calcDutyCycle(servo1,0);
    dutyCycle[1] = calcDutyCycle(servo2,0);
    dutyCycle[6] = calcDutyCycle(servo9,0);
    dutyCycle[5] = calcDutyCycle(servo8,0);
  
    PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
    PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);

    delay(1000);

  }

}

//-----------------------------------------------------------------------------------------------------//
/*
#define echoPin 16
#define trigPin 17

long sonarDuration;
int distance;

LIS3DH myIMU; //Default constructor is I2C, addr 0x19.
//this connects to pins 4 and 5 , dk why

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);
  pinMode(15,OUTPUT);



  //Call .begin() to configure the IMU
  myIMU.begin();

}


void loop()
{
  //Get all parameters
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatAccelX(), 4);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatAccelY(), 4);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatAccelZ(), 4);

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  sonarDuration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = sonarDuration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");


  if(distance < 10 && distance != 0 ){
    digitalWrite(15,HIGH);
  }
  delay(1000);
  digitalWrite(15,LOW);
  delay(1500);


}*/