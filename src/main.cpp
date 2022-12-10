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

fsm_t creep_forward, rotate_left, wiggle, adjust;

unsigned long interval, last_cycle = 0;
unsigned long loop_micros;

void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state changed tis is reset
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

unsigned long read_pulse(int pin)
{
    static unsigned long rising_time;  // time of the rising edge
    static int last_state;             // previous pin state
    int state = digitalRead(pin);      // current pin state
    unsigned long pulse_length = 0;    // default return value

    // On rising edge: record current time.
    if (last_state == LOW && state == HIGH) {
        rising_time = micros();
    }

    // On falling edge: report pulse length.
    if (last_state == HIGH && state == LOW) {
        unsigned long falling_time = micros();
        pulse_length = falling_time - rising_time;
    }

    last_state = state;
    return pulse_length;
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
      sonarDuration = read_pulse(echoPin);
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
    Serial.println(fx);

    angle = asin(fy/sqrt(pow(fx,2) + pow(fy,2) + pow(fz,2) ) ) * 180.0 / 3.14;

    return angle;

}

double getRoll(){

    //returns pitch in degrees

    double angle;
    double fz,fx,fy;

    fx = myIMU.readFloatAccelX();
    fz = myIMU.readFloatAccelZ();
    fy = myIMU.readFloatAccelY();
    angle = acos(fx/sqrt(pow(fx,2) + pow(fy,2) + pow(fz,2) ) ) * 180.0 / 3.14;

    return angle - 90;

}

void rot_l(){
  if(rotate_left.state == 0 && rotate_left.tis > 100){
   
    rotate_left.new_state = 1;
  }
  else if(rotate_left.state == 1 && rotate_left.tis > 100){
   
    rotate_left.new_state = 2;
  }
  else if(rotate_left.state == 2 && rotate_left.tis > 100){
   
    rotate_left.new_state = 3;
  }
  else if(rotate_left.state == 3 && rotate_left.tis > 100){

    rotate_left.new_state = 4;
  }
  else if(rotate_left.state == 4 && rotate_left.tis > 100){

    rotate_left.new_state = 0;
  }else if(rotate_left.state == 10 && adjust.state == 1){
    rotate_left.new_state = 0;
  }else if(rotate_left.state > 0 && adjust.state == 0){
    rotate_left.new_state = 10;
  }
  
}

int cont = 0;
int cont2 = 0;

void wig(){
  if(wiggle.state == 0 && wiggle.tis > 1000){
    wiggle.new_state = 1;
  }else if(wiggle.state == 1 && wiggle.tis > 500){
    wiggle.new_state = 2;
  }else if(wiggle.state == 2 && wiggle.tis > 500){
    wiggle.new_state = 3;
    cont++;
  }else if(wiggle.state == 3 && cont == 7){
    wiggle.new_state = 4;
  }else if(wiggle.state == 4 && wiggle.tis > 300){
    wiggle.new_state = 5;
    cont = 0;
  }else if(wiggle.state == 3 && wiggle.tis > 100 && cont < 7){
    wiggle.new_state = 2;
  }else if(wiggle.state == 5 && wiggle.tis > 200 && cont < 10){
    wiggle.new_state = 6;
    cont2++;
  }else if(wiggle.state == 6 && wiggle.tis > 200 && cont < 10){
    wiggle.new_state = 5;
  }else if( (wiggle.state == 5 || wiggle.state == 6) && cont2 > 10){
    wiggle.new_state = 0;
    cont2 = 0;
  }
}

void adj(){
  if(adjust.state == 0 && adjust.tis > 12000){
    adjust.new_state = 1;
  }else if(adjust.state == 1 && adjust.tis > 400){
    adjust.new_state = 0;
  }
}

float frequency = 50;
int wiggleval;
double thetaP, thetaR;

void setup()
{
  //Accel SCL-GPIO5
  //Accel SDA-GPIO4
  //Accel VCC-3.3V
 
  interval = 10;
  

  //uint32_t PWM_Pins[]       = { servo1, servo2, servo5, servo6, servo7, servo8, servo9, servo10 };

  pinMode(27,OUTPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);

  for (uint8_t index = 0; index < 8; index++)
  {
    PWM_Instance[index] = new RP2040_PWM(PWM_Pins[index], frequency, dutyCycle[index]);
  }

  set_state(creep_forward,0);
  set_state(rotate_left,10);  //not working right, careful when calling rot_l and setstate in loop
  set_state(wiggle,10);
  set_state(adjust,0);

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
    Serial.println("Roll: ");
    Serial.println(thetaR,2);
    
    Serial.println(distance);

    unsigned long cur_time = millis();
    creep_forward.tis = cur_time - creep_forward.tes;
    rotate_left.tis = cur_time - rotate_left.tes;
    wiggle.tis = cur_time - wiggle.tes;
    adjust.tis = cur_time - adjust.tes;
    
    
    //uint32_t PWM_Pins[]       = { servo1, servo2, servo5, servo6, servo7, servo8, servo9, servo10 }; para ver quais servos em que pinos
    //servo 9 together w 10, servo6 w 8
    //servo testing
    
    rot_l();
    wig();
    adj();

    if(creep_forward.state == 0 && creep_forward.tis > 200 && adjust.state != 1){

      creep_forward.new_state = 1;

    }else if(creep_forward.state == 1 && creep_forward.tis > 100 && adjust.state != 1){

      creep_forward.new_state = 7;

    }else if(creep_forward.state == 7 && creep_forward.tis > 200 && adjust.state != 1){
      creep_forward.new_state = 2;
    }
    else if(creep_forward.state == 2 && creep_forward.tis > 200 && adjust.state != 1){

      creep_forward.new_state = 3;

    }else if(creep_forward.state == 8 && creep_forward.tis > 200 && adjust.state != 1){

      creep_forward.new_state = 4;

    }
    else if(creep_forward.state == 3 && creep_forward.tis > 100 && adjust.state != 1){

      creep_forward.new_state = 8;

    }else if(creep_forward.state == 4 && creep_forward.tis > 100 && adjust.state != 1){

      creep_forward.new_state = 9;

    }else if(creep_forward.state == 9 && creep_forward.tis > 200 && adjust.state != 1){

      creep_forward.new_state = 5;

    }
    else if(creep_forward.state == 5 && creep_forward.tis > 300 && adjust.state != 1){

      creep_forward.new_state = 6;

    }else if(creep_forward.state == 6 && creep_forward.tis > 80 && adjust.state != 1){

      creep_forward.new_state = 10;

    }else if(creep_forward.state == 10 && creep_forward.tis > 100 && adjust.state != 1){

      creep_forward.new_state = 0;

    }else if(creep_forward.state > 0 && adjust.state == 1){
      creep_forward.new_state = 50;
    }else if(creep_forward.state == 50 && adjust.state == 0){
      creep_forward.new_state = 0;
    }

    set_state(creep_forward,creep_forward.new_state);
    set_state(rotate_left,rotate_left.new_state); // change to rotate_left.new_state
    set_state(wiggle,10);
    set_state(adjust,adjust.new_state);

    //creep_forward outputs
    if(creep_forward.state == 0){
      dutyCycle[1] = calcDutyCycle(servo2,90);
      dutyCycle[6] = calcDutyCycle(servo9,90);
      dutyCycle[5] = calcDutyCycle(servo8,45);
      dutyCycle[0] = calcDutyCycle(servo1,45);
      dutyCycle[4] = calcDutyCycle(servo7,0);
      PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]); 
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
      
    }else if(creep_forward.state == 1){
      dutyCycle[7] = calcDutyCycle(servo10,50);
      dutyCycle[6] = calcDutyCycle(servo9,20);
      dutyCycle[1] = calcDutyCycle(servo2,90);
      dutyCycle[5] = calcDutyCycle(servo8,45);
      dutyCycle[0] = calcDutyCycle(servo1,45);
      PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]); 
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    }else if(creep_forward.state == 2){
      
      dutyCycle[6] = calcDutyCycle(servo9,45);
      dutyCycle[1] = calcDutyCycle(servo2,45);
      dutyCycle[5] = calcDutyCycle(servo8,20);
      dutyCycle[0] = calcDutyCycle(servo1,90);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]); 
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    }else if(creep_forward.state == 3){
      dutyCycle[3] = calcDutyCycle(servo6,50);
      dutyCycle[6] = calcDutyCycle(servo9,45);
      dutyCycle[1] = calcDutyCycle(servo2,45);
      dutyCycle[5] = calcDutyCycle(servo8,90);
      dutyCycle[0] = calcDutyCycle(servo1,90);
      PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]); 
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    }else if(creep_forward.state == 4){
      dutyCycle[2] = calcDutyCycle(servo5,50);
      dutyCycle[0] = calcDutyCycle(servo1,10);
      dutyCycle[5] = calcDutyCycle(servo8,90);
      dutyCycle[6] = calcDutyCycle(servo9,45);
      dutyCycle[1] = calcDutyCycle(servo2,45);
      PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]); 
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    }else if(creep_forward.state == 5){
      dutyCycle[0] = calcDutyCycle(servo1,45);
      dutyCycle[5] = calcDutyCycle(servo8,45);
      dutyCycle[6] = calcDutyCycle(servo9,90);
      dutyCycle[1] = calcDutyCycle(servo2,20);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]); 
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    }else if(creep_forward.state == 6){
      dutyCycle[0] = calcDutyCycle(servo1,45);
      dutyCycle[4] = calcDutyCycle(servo7,50);
      dutyCycle[5] = calcDutyCycle(servo8,45);
      dutyCycle[6] = calcDutyCycle(servo9,90);
      dutyCycle[1] = calcDutyCycle(servo2,90);
      PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]); 
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    }else if(creep_forward.state == 7){
      dutyCycle[7] = calcDutyCycle(servo10,0);
      PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    }else if(creep_forward.state == 8){
      dutyCycle[3] = calcDutyCycle(servo6,0);
      PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);
    }else if(creep_forward.state == 9){
      dutyCycle[2] = calcDutyCycle(servo5,0);
      PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
    }else if(creep_forward.state == 10){
      dutyCycle[4] = calcDutyCycle(servo7,0);
      PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
    }
     
     //rotate left outputs

  if(rotate_left.state == 0){
    dutyCycle[7] = calcDutyCycle(servo10,0);
    dutyCycle[6] = calcDutyCycle(servo9,45);  
    dutyCycle[3] = calcDutyCycle(servo6,0);  
    dutyCycle[5] = calcDutyCycle(servo8,45);  
    dutyCycle[0] = calcDutyCycle(servo1,45);
    dutyCycle[2] = calcDutyCycle(servo5,0);
    dutyCycle[1] = calcDutyCycle(servo2,45);
    dutyCycle[4] = calcDutyCycle(servo7,0);
    PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
    PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
    PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);
    PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);
    PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
  }
 
  else if(rotate_left.state == 1){
    dutyCycle[7] = calcDutyCycle(servo10,0);
    dutyCycle[6] = calcDutyCycle(servo9,70);  //leg2 bw  
    dutyCycle[3] = calcDutyCycle(servo6,0);  
    dutyCycle[5] = calcDutyCycle(servo8,70);  //leg3 fw  
    dutyCycle[0] = calcDutyCycle(servo1,70);  //leg1 bw
    dutyCycle[2] = calcDutyCycle(servo5,20);  //leg1 up
    dutyCycle[1] = calcDutyCycle(servo2,70);  //leg4 fw
    dutyCycle[4] = calcDutyCycle(servo7,20);  //leg4 up
    PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
    PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
    PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);
    PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);
    PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
  }

  else if(rotate_left.state == 2){
    dutyCycle[7] = calcDutyCycle(servo10,0);
    dutyCycle[6] = calcDutyCycle(servo9,70);    
    dutyCycle[3] = calcDutyCycle(servo6,0);  
    dutyCycle[5] = calcDutyCycle(servo8,70);  
    dutyCycle[0] = calcDutyCycle(servo1,70);  
    dutyCycle[2] = calcDutyCycle(servo5,0);   //leg1 dn
    dutyCycle[1] = calcDutyCycle(servo2,70);  
    dutyCycle[4] = calcDutyCycle(servo7,0);   //leg4 dn
    PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
    PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
    PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);
    PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);
    PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
  }

  else if(rotate_left.state == 3){
    dutyCycle[7] = calcDutyCycle(servo10,20); //leg2 up
    dutyCycle[6] = calcDutyCycle(servo9,20);  //leg2 fw  
    dutyCycle[3] = calcDutyCycle(servo6,20);  //leg3 up  
    dutyCycle[5] = calcDutyCycle(servo8,20);  //leg3 bw
    dutyCycle[0] = calcDutyCycle(servo1,20);  //leg1 fw
    dutyCycle[2] = calcDutyCycle(servo5,0);
    dutyCycle[1] = calcDutyCycle(servo2,20);  //leg4 bw
    dutyCycle[4] = calcDutyCycle(servo7,0);
    PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
    PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
    PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);
    PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);
    PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
  }
  else if(rotate_left.state == 4){
    dutyCycle[7] = calcDutyCycle(servo10,0); //leg2 dn
    dutyCycle[6] = calcDutyCycle(servo9,20);  
    dutyCycle[3] = calcDutyCycle(servo6,0);  //leg3 dn  
    dutyCycle[5] = calcDutyCycle(servo8,20);  
    dutyCycle[0] = calcDutyCycle(servo1,20);  
    dutyCycle[2] = calcDutyCycle(servo5,0);
    dutyCycle[1] = calcDutyCycle(servo2,20);  
    dutyCycle[4] = calcDutyCycle(servo7,0);
    PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
    PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
    PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);
    PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);
    PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
  }

  //wiggle state

  if(wiggle.state == 0){
    dutyCycle[7] = calcDutyCycle(servo10,0); //leg2 dn
    dutyCycle[6] = calcDutyCycle(servo9,80);  
    dutyCycle[3] = calcDutyCycle(servo6,0);  //leg3 dn  
    dutyCycle[5] = calcDutyCycle(servo8,80);  
    dutyCycle[0] = calcDutyCycle(servo1,80);  
    dutyCycle[2] = calcDutyCycle(servo5,0);
    dutyCycle[1] = calcDutyCycle(servo2,80);  
    dutyCycle[4] = calcDutyCycle(servo7,0);
    PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
    PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
    PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);
    PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);
    PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
    wiggleval = 80;
  }else if(wiggle.state == 1){
    dutyCycle[0] = calcDutyCycle(servo1,20);
    dutyCycle[6] = calcDutyCycle(servo9,20);
    PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
    PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);

  }else if(wiggle.state == 2){
    dutyCycle[2] = calcDutyCycle(servo5,70);
    dutyCycle[7] = calcDutyCycle(servo10,70);
    PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
  }else if(wiggle.state == 3 && wiggleval > 10){
    wiggleval -= 10;
    dutyCycle[5] = calcDutyCycle(servo8,wiggleval);  
    dutyCycle[1] = calcDutyCycle(servo2,wiggleval);  
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
  }else if(wiggle.state == 5){
    dutyCycle[5] = calcDutyCycle(servo8,60);
    dutyCycle[1] = calcDutyCycle(servo2,60);
    dutyCycle[2] = calcDutyCycle(servo5,70);
    dutyCycle[7] = calcDutyCycle(servo10,70); //leg2 dn
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
    PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
  }else if(wiggle.state == 6){
    dutyCycle[5] = calcDutyCycle(servo8,10);
    dutyCycle[1] = calcDutyCycle(servo2,10);
    dutyCycle[2] = calcDutyCycle(servo5,0);
    dutyCycle[7] = calcDutyCycle(servo10,0); //leg2 dn
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
    PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
  }

   
    Serial.print("rotate left state: ");
    Serial.println(rotate_left.new_state);
    Serial.print("creep forward state: ");
    Serial.println(creep_forward.new_state);
    Serial.print("wiggle state: ");
    Serial.println(wiggle.new_state);
    Serial.print("Contador2: ");
    Serial.print(cont2);
    
    /*
    dutyCycle[2] = calcDutyCycle(servo5,30); // servo 5
    dutyCycle[4] = calcDutyCycle(servo7,30);
    dutyCycle[7] = calcDutyCycle(servo10,30);
    dutyCycle[3] = calcDutyCycle(servo6,30);
    PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]); 
    PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]); 
    PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]); 
    PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);


    //dutyCycle[6] = calcDutyCycle(servo9,65); //servo8
    
    dutyCycle[5] = calcDutyCycle(servo8,25); //servo7
    dutyCycle[0] = calcDutyCycle(servo1,65);
    dutyCycle[1] = calcDutyCycle(servo2,25);
    PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]); //servo8
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]); //servo7
    //PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]); 
    //PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);*/ 
    
    
    

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