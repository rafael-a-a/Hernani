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
#define NUM_SAMPLES 300
#define alpha 0.5


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
uint32_t distance32;
/*
// Output variables

// Our finite state machines
fsm_t fsm1, fsm2, fsm3, fsm4, fsm5, fsm6, fsm7, fsm8, fsm9, fsm10, fsm11;
*/

fsm_t creep_forward, rotate_left,rotate_right, wiggle, adjust, self_balance, sonar_core1,creep_backward,main_controller;

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

  if(angle < 0){
    angle = 0;
  }
  if(angle > 160){
    angle = 160;
  }

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

//variables for interrupt(sonar)
uint32_t echo_end,trig_beg;
int event_trig = 0;

float getDistance(){

  //returns distance from nearest object in cm
  
  int state=0;
  long auxtemp=micros();
  float distance=0;

  digitalWrite(trigPin,LOW);

  while (auxtemp - micros() < 25){

    if(state == 0 && micros() - auxtemp > 2){
      
      state = 1;
      digitalWrite(trigPin,HIGH);
      auxtemp=micros();
      

    } else if (state == 1 && micros() - auxtemp > 10){
      
      digitalWrite(trigPin,LOW);
      trig_beg = micros();
      auxtemp = micros();
      state = 0;
      /*sonarDuration = read_pulse(echoPin);
      distance = sonarDuration * 0.034 / 2;
      auxtemp = micros();
      state = 0;*/
    }
    
  }
  return 1;

}

double getPitch(){

    //returns pitch in degrees

    double angle;
    double fz,fx,fy;

    fx = myIMU.readFloatAccelX();
    fz = myIMU.readFloatAccelZ();
    fy = myIMU.readFloatAccelY();
    

    angle = asin(fy/sqrt(pow(fx,2) + pow(fy,2) + pow(fz,2) ) ) * 180.0 / 3.14;

    return angle;

}

double getRoll(){

    //returns roll in degrees

    double angle;
    double fz,fx,fy;

    fx = myIMU.readFloatAccelX();
    fz = myIMU.readFloatAccelZ();
    fy = myIMU.readFloatAccelY();
    angle = acos(fx/sqrt(pow(fx,2) + pow(fy,2) + pow(fz,2) ) ) * 180.0 / 3.14;

    return angle - 90;

}

int random_int = 0;//to determine if robot turns left(0) or right(1) when obstacle is detected

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
  }else if(rotate_left.state == 10 && ( (adjust.state == 1 && creep_forward.state == 50) || (main_controller.state == 2 && random_int == 0)) ){
    rotate_left.new_state = 0;
  }else if(rotate_left.state > 0 && adjust.state == 0){
    rotate_left.new_state = 10;
  }else if(self_balance.state == 0 || (adjust.state == 10 && main_controller.state != 1) ){
    rotate_left.new_state = 10;
  }
  
}

void rot_r(){
   if(rotate_right.state == 0 && rotate_right.tis > 100){
   
    rotate_right.new_state = 1;
  }
  else if(rotate_right.state == 1 && rotate_right.tis > 100){
   
    rotate_right.new_state = 2;
  }
  else if(rotate_right.state == 2 && rotate_right.tis > 100){
   
    rotate_right.new_state = 3;
  }
  else if(rotate_right.state == 3 && rotate_right.tis > 100){

    rotate_right.new_state = 4;
  }
  else if(rotate_right.state == 4 && rotate_right.tis > 100){

    rotate_right.new_state = 0;
  }else if(rotate_right.state == 10  && ( (adjust.state == 1 && creep_backward.state == 50) || (main_controller.state == 2 && random_int == 1))){

    rotate_right.new_state = 0;
  }else if(rotate_right.state > 0 && adjust.state == 0 ){

    rotate_right.new_state = 10;
  }else if(self_balance.state == 0 || (adjust.state == 10 && main_controller.state != 1)){

    rotate_right.new_state = 10;
  }
}

void c_backward(){  //change this to include main controller, also change creep_forward
  if(creep_backward.state == 0 && creep_backward.tis > 200){

      creep_backward.new_state = 1;

    }else if(creep_backward.state == 1 && creep_backward.tis > 100){

      creep_backward.new_state = 7;

    }else if(creep_backward.state == 7 && creep_backward.tis > 200){
      creep_backward.new_state = 2;
    }
    else if(creep_backward.state == 2 && creep_backward.tis > 200){

      creep_backward.new_state = 3;

    }else if(creep_backward.state == 8 && creep_backward.tis > 200){

      creep_backward.new_state = 4;

    }
    else if(creep_backward.state == 3 && creep_backward.tis > 100){

      creep_backward.new_state = 8;

    }else if(creep_backward.state == 4 && creep_backward.tis > 100){

      creep_backward.new_state = 9;

    }else if(creep_backward.state == 9 && creep_backward.tis > 200){

      creep_backward.new_state = 5;

    }
    else if(creep_backward.state == 5 && creep_backward.tis > 100){

      creep_backward.new_state = 6;

    }else if(creep_backward.state == 6 && creep_backward.tis > 80){

      creep_backward.new_state = 10;

    }else if(creep_backward.state == 10 && creep_backward.tis > 100){

      creep_backward.new_state = 0;

    }else if(self_balance.state == 0 || main_controller.state == 1){ // if the robot is balancing/turning, it cant be moving
      creep_backward.new_state = 50;
    }else if(creep_backward.state == 50 && self_balance.state != 0 && main_controller.state == 0){
      creep_backward.new_state = 0;
    }
}

//comparations of distance32 w/ 0 are because sometimes the sensor bugs and reads 0
void m_cont(){

  if(main_controller.state == 0 && distance32 <= 7 && distance32 != 0){ // when state = 0, walks(creep_backward)
    main_controller.new_state = 1; //go back
    randomSeed(millis());
    random_int = random(2);

  }else if(main_controller.state == 1 && main_controller.tis > 800){
    main_controller.new_state = 2; // turn left or right

  }else if(main_controller.state == 2 && main_controller.tis > 600 && (distance32 > 15 || distance32 == 0)){ //when state = 1, turns left or right
    main_controller.new_state = 0; //after turning, if there's space in front, go back to walk
  }else if(main_controller.state == 50){
      main_controller.new_state = 50;
  }
  
  //in state 0 does nothing, in state 1 the robot should be moving backwards(creep_forward), in state 2 the robot is turning for a minimum of 600ms
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

//fsm for adjusting to the left
void adj(){
  if(adjust.state == 0 && adjust.tis > 6000){
    adjust.new_state = 1;
  }else if(adjust.state == 1 && adjust.tis > 400){
    adjust.new_state = 0;
  }else if(self_balance.state == 0 || main_controller.state == 1){  //dont adjust if its turning
    adjust.new_state = 10;
  }
}

uint32_t echo_beg = 0;
void echo_rising_edge(){
  if(digitalRead(echoPin)){
    echo_beg = micros();
  }
  if(!digitalRead(echoPin)){
    echo_end = micros();
    distance = (echo_end - echo_beg) / 1e3 * 0.034 / 2;
  }

  event_trig = 1;

}


float frequency = 50;
int wiggleval;
double thetaP, thetaR;

/*For balance(PID) control*/

//constants
float roll_kp = 0.1, roll_kd = 80, roll_ki = 0.00000;
float pitch_kp = 0.05, pitch_kd = 50, pitch_ki = 0.0;
//for delta time use cur_time
float dt;

//setpoint
float rollTarget = 0, rollError = 0, rollServoValLeft = 0, rollServoValRight = 0;
float rollErrorOld, rollErrorChange, rollErrorSlope;
float rollErrorArea = 0;

float pitchTarget = 0, pitchError = 0, pitchServoValLeft = 0, pitchServoValRight = 0;
float pitchErrorOld, pitchErrorChange, pitchErrorSlope;
float pitchErrorArea = 0;

float servolVal_6 = 0, servolVal_7 = 0, servolVal_5 = 0, servolVal_10 = 0;

//values set to 0 to make sure the controller is able to work
// when the system is booted

//Pitch -> 90º to front, -90 to back
//5,6 +1 for roll
//7,10 -1 for roll

//matrix of influence
int M[2][4] = {{1,-1,-1,1},{1,1,-1,-1}};
/*      
  Servo    6     7    10     5
   Roll   |1    -1    -1     1|
   Pitch  |1     1    -1    -1|
   
*/


void setup()
{
  //Accel SCL-GPIO5
  //Accel SDA-GPIO4
  //Accel VCC-3.3V
 
  interval = 10;
  

  //uint32_t PWM_Pins[]       = { servo1, servo2, servo5, servo6, servo7, servo8, servo9, servo10 };

  pinMode(27,OUTPUT);
  //RIGHT NOW TRIG AND ECHO ARE BEING DECLARED IN CORE1
  /*
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);
  */
  for (uint8_t index = 0; index < 8; index++)
  {
    PWM_Instance[index] = new RP2040_PWM(PWM_Pins[index], frequency, dutyCycle[index]);
  }

  set_state(creep_forward,50); //0 to walk, 50 to turn off
  set_state(rotate_left,10);  //not working right, careful when calling rot_l and setstate in loop
  set_state(wiggle,10);       //0 to wiggle, 10 to turn off
  set_state(adjust,0);        //0 to walk, 10 to turn off
  set_state(self_balance,1); //0 to balance, 1 to disable
  set_state(sonar_core1,0);
  set_state(creep_backward,00); //0 to walk, 50 to turn off, this is actually walking forward, not backward
  set_state(rotate_right,10);
  set_state(main_controller,0); //controls the obstacle avoidance

  //attachInterrupt(echoPin,echo_rising_edge,CHANGE);

  myIMU.begin();
  
 
}

//setup for second core

void setup1(){

  pinMode(echoPin,INPUT);
  pinMode(trigPin,OUTPUT);
  delay(800);

}


int flag = 0;

//loop for second core
void loop1(){

  
  static unsigned long last_cycle1 = 0;

  unsigned long curr_time1 = millis();
  if (curr_time1 - last_cycle1 >= 50) {
    // Send a pulse to the trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure the time it takes for the pulse to return
    long duration1 = pulseIn(echoPin, HIGH);

    // Calculate the distance based on the speed of sound and the time it took for the pulse to return
    int distance1 = duration1 * 0.034 / 2;

    // Write the distance to the FIFO buffer
    rp2040.fifo.push_nb(distance1);
    
    // Update the last measurement time
    last_cycle1 = curr_time1;
  }

    
    
}




//Filter for accel
float samplesAccRoll[NUM_SAMPLES] = {0}, samplesAccPitch[NUM_SAMPLES] = {0};
float sumRoll = 0, sumPitch = 0;
int indexRoll = 0, indexPitch = 0;
float finalRoll, finalPitch = 0;

void loop()
{
  unsigned long now = millis();
  

  if(now - last_cycle > interval){

  
    //update inputs
    
    //this function does not change the value in distance32 if fifo is empty, and never blocks the execution of the program
    rp2040.fifo.pop_nb(&distance32);
    
    thetaP = getPitch();
    thetaR = getRoll();

    Serial.println("Distance");
    Serial.println(distance32);
    /*
    

    
    Serial.println("Pitch:");
    Serial.println(thetaP, 2);
    Serial.println("Roll: ");
    Serial.println(thetaR,2);
    */
   

    sumRoll -= samplesAccRoll[indexRoll]; //subtract oldest measurement
    sumRoll += thetaR; //add to total new measurement
    samplesAccRoll[indexRoll] = thetaR; //save new measurement
    indexRoll = (indexRoll + 1) % NUM_SAMPLES; //increase counter (circular array)

    //finalRoll = sumRoll / NUM_SAMPLES; //MAF
    finalRoll = (1 - alpha) * sumRoll / NUM_SAMPLES + alpha * thetaR;   //exponentially weighted moving average (EWMA) 

    sumPitch -= samplesAccPitch[indexPitch];
    sumPitch += thetaP;
    samplesAccPitch[indexPitch] = thetaP;
    indexPitch = (indexPitch + 1) % NUM_SAMPLES;
    finalPitch = (1 - alpha) * sumPitch / NUM_SAMPLES + alpha * thetaP;

    dt = now - last_cycle;
    rollErrorOld = rollError;
    rollError = rollTarget - finalRoll;
    rollErrorChange = rollError - rollErrorOld;
    rollErrorSlope = rollErrorChange / dt;
    rollErrorArea = rollErrorArea + rollError * dt;

    pitchErrorOld = pitchError;
    pitchError = pitchTarget - thetaP;
    pitchErrorChange = pitchError - pitchErrorOld;
    pitchErrorSlope = pitchErrorChange / dt;
    pitchErrorArea = pitchErrorArea + pitchError * dt;

   // Left: 5,6 (switched)
   // Right: 7,10
   // Front 7,6
   // Back 10,5

   /*   
   Servo    6    7    10   5
   Roll   |1    -1    -1     1|
   Pitch  |1     1    -1    -1|

   */

    servolVal_10 = servolVal_10 + M[0][2] * (roll_kp * rollError + roll_kd * rollErrorSlope + roll_ki * rollErrorArea) + M[1][2] * (pitch_kp * pitchError + pitch_kd * pitchErrorSlope + pitch_ki * pitchErrorArea);
    servolVal_7 = servolVal_7 + M[0][1] * (roll_kp * rollError + roll_kd * rollErrorSlope + roll_ki * rollErrorArea) + M[1][1] * (pitch_kp * pitchError + pitch_kd * pitchErrorSlope + pitch_ki * pitchErrorArea);
    servolVal_6 = servolVal_6 + M[0][0] * (roll_kp * rollError + roll_kd * rollErrorSlope + roll_ki * rollErrorArea) + M[1][0] * (pitch_kp * pitchError + pitch_kd * pitchErrorSlope + pitch_ki * pitchErrorArea);
    servolVal_5 = servolVal_5 + M[0][3] * (roll_kp * rollError + roll_kd * rollErrorSlope + roll_ki * rollErrorArea) + M[1][3] * (pitch_kp * pitchError + pitch_kd * pitchErrorSlope + pitch_ki * pitchErrorArea);
     
    if(servolVal_10 < 0){
      servolVal_10 = 0;
    }
    if(servolVal_10 > 68){
      servolVal_10 = 68;
    }
    if(servolVal_6 < 0){
      servolVal_6 = 0;
    }
    if(servolVal_6 > 68){
      servolVal_6 = 68;
    }
    if(servolVal_5 < 0){
      servolVal_5 = 0;
    }
    if(servolVal_5 > 68){
      servolVal_5 = 68;
    }
    if(servolVal_7 < 0){
      servolVal_7 = 0;
    }
    if(servolVal_7 > 68){
      servolVal_7 = 68;
    }


    unsigned long cur_time = millis();
    creep_forward.tis = cur_time - creep_forward.tes;
    rotate_left.tis = cur_time - rotate_left.tes;
    wiggle.tis = cur_time - wiggle.tes;
    adjust.tis = cur_time - adjust.tes;
    sonar_core1.tis = cur_time - sonar_core1.tes;
    creep_backward.tis = cur_time - creep_backward.tes;
    rotate_right.tis = cur_time - rotate_right.tes;
    main_controller.tis = cur_time - main_controller.tes;
    
    
    //uint32_t PWM_Pins[]       = { servo1, servo2, servo5, servo6, servo7, servo8, servo9, servo10 }; para ver quais servos em que pinos
    //servo 9 together w 10, servo6 w 8
    //servo testing
    
    rot_l();
    wig();
    adj();
    c_backward();
    rot_r();
    m_cont();

    if(creep_forward.state == 0 && creep_forward.tis > 200 && adjust.state != 1){

      creep_forward.new_state = 1;

    }else if(self_balance.state == 0 || main_controller.state == 1){     //IF SELFBALANCE/turning ACTIVE, DOESNT CREEP!!!!
      creep_forward.new_state = 50;
    }
    else if(creep_forward.state == 1 && creep_forward.tis > 100 && adjust.state != 1){

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

    }else if(creep_forward.state > 0 && (adjust.state == 1 || creep_backward.state != 50) ){
      creep_forward.new_state = 50;
    }else if(creep_forward.state == 50 && adjust.state == 0 && self_balance.state == 1 && (main_controller.state == 0 || main_controller.state == 1) && creep_backward.state == 50){
      creep_forward.new_state = 0;
    }

    if(sonar_core1.state == 0 && sonar_core1.tis > 50){
      sonar_core1.new_state = 1;
    }else if(sonar_core1.state == 1 && sonar_core1.tis > 5){
      sonar_core1.new_state = 0;
    }

    set_state(creep_forward,creep_forward.new_state);
    set_state(rotate_left,rotate_left.new_state);
    set_state(wiggle,10); //dont forget to change if want to wiggle
    set_state(adjust,adjust.new_state);
    set_state(sonar_core1,sonar_core1.new_state);
    set_state(creep_backward,creep_backward.new_state);
    set_state(rotate_right,rotate_right.new_state);
    set_state(main_controller,main_controller.new_state);

    if(sonar_core1.state == 0){
      flag = 0;
    }else if(sonar_core1.state == 1){
      flag = 1;
    }

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

  //wiggle outputs

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

  //rotate right
  if(rotate_right.state == 0){
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
  else if(rotate_right.state == 1){
    dutyCycle[7] = calcDutyCycle(servo10,20); //leg2 up
    dutyCycle[6] = calcDutyCycle(servo9,70);  //leg2 bw
    dutyCycle[3] = calcDutyCycle(servo6,20);  //leg3 up  
    dutyCycle[5] = calcDutyCycle(servo8,70);  //leg3 fw
    dutyCycle[0] = calcDutyCycle(servo1,70);  //leg1 bw
    dutyCycle[2] = calcDutyCycle(servo5,0);
    dutyCycle[1] = calcDutyCycle(servo2,70);  //leg4 fw
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
  else if(rotate_right.state == 2){
    dutyCycle[7] = calcDutyCycle(servo10,0);  //leg2 dn
    dutyCycle[6] = calcDutyCycle(servo9,70);  
    dutyCycle[3] = calcDutyCycle(servo6,0);   //leg3 dn
    dutyCycle[5] = calcDutyCycle(servo8,70);  
    dutyCycle[0] = calcDutyCycle(servo1,70);  
    dutyCycle[2] = calcDutyCycle(servo5,0);
    dutyCycle[1] = calcDutyCycle(servo2,70);  
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
  else if(rotate_right.state == 3){
    dutyCycle[7] = calcDutyCycle(servo10,0);  
    dutyCycle[6] = calcDutyCycle(servo9,20);  //leg2 fw
    dutyCycle[3] = calcDutyCycle(servo6,0);  
    dutyCycle[5] = calcDutyCycle(servo8,20);  //leg3 bw
    dutyCycle[0] = calcDutyCycle(servo1,20);  //leg1 fw
    dutyCycle[2] = calcDutyCycle(servo5,20);  //leg1 up
    dutyCycle[1] = calcDutyCycle(servo2,20);  //leg4 bw
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
  else if(rotate_right.state == 4){
    dutyCycle[7] = calcDutyCycle(servo10,0);  
    dutyCycle[6] = calcDutyCycle(servo9,20);  
    dutyCycle[3] = calcDutyCycle(servo6,0);  
    dutyCycle[5] = calcDutyCycle(servo8,20);  
    dutyCycle[0] = calcDutyCycle(servo1,20);  
    dutyCycle[2] = calcDutyCycle(servo5,0);  //leg1 dn
    dutyCycle[1] = calcDutyCycle(servo2,20);  
    dutyCycle[4] = calcDutyCycle(servo7,0);  //leg4 dn
    PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
    PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
    PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);
    PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);
    PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
  }

  //balance state

  if(self_balance.state == 0 ){
    //this first block is to stabilize Hernani
    dutyCycle[5] = calcDutyCycle(servo8,45);
    dutyCycle[1] = calcDutyCycle(servo2,45);
    dutyCycle[0] = calcDutyCycle(servo1,45);
    dutyCycle[6] = calcDutyCycle(servo9,45);
    //this is the PID control
    dutyCycle[2] = calcDutyCycle(servo5,servolVal_5);
    dutyCycle[3] = calcDutyCycle(servo6,servolVal_6);
    dutyCycle[7] = calcDutyCycle(servo10,servolVal_10);
    dutyCycle[4] = calcDutyCycle(servo7,servolVal_7);

    PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
    PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);
    PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
    PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
    PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
    PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);
  }

  //creep_backward output ()

  if(creep_backward.state == 0){
      dutyCycle[0] = calcDutyCycle(servo1,90);
      dutyCycle[5] = calcDutyCycle(servo8,90);
      dutyCycle[6] = calcDutyCycle(servo9,45);
      dutyCycle[1] = calcDutyCycle(servo2,45);
      dutyCycle[2] = calcDutyCycle(servo5,0);
      PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
     
    }else if(creep_backward.state == 1){
      dutyCycle[3] = calcDutyCycle(servo6,50);
      dutyCycle[5] = calcDutyCycle(servo8,20);
      dutyCycle[0] = calcDutyCycle(servo1,90);
      dutyCycle[6] = calcDutyCycle(servo9,45);
      dutyCycle[1] = calcDutyCycle(servo2,45);
      PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    }else if(creep_backward.state == 2){
     
      dutyCycle[5] = calcDutyCycle(servo8,45);
      dutyCycle[0] = calcDutyCycle(servo1,45);
      dutyCycle[6] = calcDutyCycle(servo9,20);
      dutyCycle[1] = calcDutyCycle(servo2,90);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    }else if(creep_backward.state == 3){
      dutyCycle[7] = calcDutyCycle(servo10,50);
      dutyCycle[5] = calcDutyCycle(servo8,45);
      dutyCycle[0] = calcDutyCycle(servo1,45);
      dutyCycle[6] = calcDutyCycle(servo9,90);
      dutyCycle[1] = calcDutyCycle(servo2,90);
      PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[9]);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    }else if(creep_backward.state == 4){
      dutyCycle[4] = calcDutyCycle(servo7,50);
      dutyCycle[1] = calcDutyCycle(servo2,10);
      dutyCycle[6] = calcDutyCycle(servo9,90);
      dutyCycle[5] = calcDutyCycle(servo8,45);
      dutyCycle[0] = calcDutyCycle(servo1,45);
      PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    }else if(creep_backward.state == 5){
      dutyCycle[1] = calcDutyCycle(servo2,45);
      dutyCycle[6] = calcDutyCycle(servo9,45);
      dutyCycle[5] = calcDutyCycle(servo8,90);
      dutyCycle[0] = calcDutyCycle(servo1,20);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    }else if(creep_backward.state == 6){
      dutyCycle[1] = calcDutyCycle(servo2,45);
      dutyCycle[2] = calcDutyCycle(servo5,50);
      dutyCycle[6] = calcDutyCycle(servo9,45);
      dutyCycle[5] = calcDutyCycle(servo8,90);
      dutyCycle[0] = calcDutyCycle(servo1,90);
      PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
      PWM_Instance[6]->setPWM(servo9, frequency, dutyCycle[6]);
      PWM_Instance[5]->setPWM(servo8, frequency, dutyCycle[5]);  
      PWM_Instance[1]->setPWM(servo2, frequency, dutyCycle[1]);
      PWM_Instance[0]->setPWM(servo1, frequency, dutyCycle[0]);
    }else if(creep_backward.state == 7){
      dutyCycle[3] = calcDutyCycle(servo6,0);
      PWM_Instance[3]->setPWM(servo6, frequency, dutyCycle[3]);
    }else if(creep_backward.state == 8){
      dutyCycle[7] = calcDutyCycle(servo10,0);
      PWM_Instance[7]->setPWM(servo10, frequency, dutyCycle[7]);
    }else if(creep_backward.state == 9){
      dutyCycle[4] = calcDutyCycle(servo7,0);
      PWM_Instance[4]->setPWM(servo7, frequency, dutyCycle[4]);
    }else if(creep_backward.state == 10){
      dutyCycle[2] = calcDutyCycle(servo5,0);
      PWM_Instance[2]->setPWM(servo5, frequency, dutyCycle[2]);
    }
   
    
   Serial.print("creep backward state:");
   Serial.println(creep_backward.state);
   Serial.print("rotate left state:");
   Serial.println(rotate_left.state);
   Serial.print("creep forward state:");
   Serial.println(creep_forward.state);
   Serial.print("adjust state:");
   Serial.println(adjust.state);
   Serial.print("rotate right state:");
   Serial.println(rotate_right.state);
   Serial.print("main controller state:");
   Serial.println(main_controller.state);
   
    
    

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

  
    if(j < 600){
      j++;
    }
    if(j >= 600 && j < 900){
      j++;
      meas[j - 600] = finalRoll;
    }
    
    
    if(j >= 900){
      Serial.println("");
      for(int i = 0; i < j - 600; i++){
        Serial.print(meas[i],2);
        Serial.print(",");
      }
      Serial.println("");
    }


}*/

