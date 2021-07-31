
/* This is a flight controller of mofified version of duddukuru flight controller for a drone with imu mpu6050 and FLYSKY PWM reciever input ,
 * This can be a test version 2.b
 * pradeep 
 */
 

#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <MPU6050.h>


//TwoWire Wire(2, I2C_FAST_MODE,400);
#define F  1000  // constrain loop frequiency 
unsigned long loop_timer ;

///////////************ IMU DATA ************//////////////
///////////////////////////////////////////////////////////
MPU6050 mpu;    // imu object 
double accPitch, accRoll ;
double pitch,roll,yaw ;
double pitch_p,roll_p;
unsigned long imutimer;
int8_t rolloffset,pitchoffset ;

//************////////// INPUT RECIEVER DATA ////////////*****************
///////////////////////////////////////////////////////////////////////
int T,Y,P,R;
int R_throttle,R_pitch,R_yaw,R_roll;
unsigned long current_count,counter_1,counter_2,counter_3,counter_4;
byte roll_state,yaw_state,pitch_state,throttle_state;
float a = 1; /// lowpass filter parameter 

//************////////// write to motors ////////////*****************
///////////////////////////////////////////////////////////////////////
double  PITCH_ERROR,PITCH_ERROR_P;
int16_t PITCH_P,PITCH_I,PITCH_D ;
int16_t PITCH_PID;

double  ROLL_ERROR,ROLL_ERROR_P;
int16_t ROLL_P,ROLL_I,ROLL_D ;
int16_t ROLL_PID ;

int16_t YAW_P;

//////////////////////// DATA TO MOTORS //////////////////////////////////
int16_t F_R_M,F_L_M,B_R_M,B_L_M ;    // motor pwm output data 
bool M = false;
/////////////////////// SERVO OBJECTS /////////////////////////////////
Servo F_R_MOTOR ;
Servo F_L_MOTOR ;
Servo B_R_MOTOR ;
Servo B_L_MOTOR ;

//***********************************************************************//

void setup() {
  
Serial.begin(115200);

pinMode(13,OUTPUT);
pinMode(11,OUTPUT);
pinMode(10,OUTPUT);
pinMode(9,OUTPUT);
pinMode(8,OUTPUT);
pinMode(5, INPUT);
pinMode(4, INPUT);
pinMode(3, INPUT);
pinMode(2, INPUT);

attachInterrupt(digitalPinToInterrupt(2),THROTTLE,CHANGE);
attachInterrupt(digitalPinToInterrupt(3),PITCH,CHANGE);


  PCICR  |= (1 << PCIE2  );    //enable PCMSK0 scan                                                 
  PCMSK2 |= (1 << PCINT20);  //Set pin D4 trigger an interrupt on state change.  yaw                                     
  PCMSK2 |= (1 << PCINT21);  //Set pin D5 trigger an interrupt on state change.  roll 

F_R_MOTOR.attach(8);
F_L_MOTOR.attach(9);
B_R_MOTOR.attach(10);
B_L_MOTOR.attach(11);


F_R_MOTOR.writeMicroseconds(1000);
F_L_MOTOR.writeMicroseconds(1000);
B_R_MOTOR.writeMicroseconds(1000);
B_L_MOTOR.writeMicroseconds(1000);


////////////////////////// caliberate motors ////////////////////
//F_R_MOTOR.writeMicroseconds(2000);
//F_L_MOTOR.writeMicroseconds(2000);
//B_R_MOTOR.writeMicroseconds(2000);
//B_L_MOTOR.writeMicroseconds(2000);

delay(2);

///////////////////////////////////////////////////////////////

Wire.begin();  // setup I2C 

inilisizeImu() ; // imu setup ....


////////////////////////////////////////////////////////////////////////////////

// activating buzzer 

     for(byte i=0;i<20;i++){
    digitalWrite(13,HIGH);
    delay(100);
    digitalWrite(13,LOW);
    delay(50);
    }
    
     digitalWrite(13,LOW);

     imutimer = millis();
}// end setup 

void loop() {
  

updateImuData(); 



if(M){
  CaliculatePid(); 
 // R = true;
}
else {
  reset_pid();
  //R = false;
}


// caliculate motor speeds 
F_R_M = map(R_throttle,1208,1920,0,32000) - PITCH_PID - ROLL_PID - YAW_P;
F_L_M = map(R_throttle,1208,1920,0,32000) - PITCH_PID + ROLL_PID + YAW_P;
B_R_M = map(R_throttle,1208,1920,0,32000) + PITCH_PID - ROLL_PID + YAW_P;
B_L_M = map(R_throttle,1208,1920,0,32000) + PITCH_PID + ROLL_PID - YAW_P;

if(F_R_M > 32000 ) F_R_M = 32000;
else if(F_R_M < 0) F_R_M = 0;

if(F_L_M > 32000 ) F_L_M = 32000;
else if(F_L_M < 0) F_L_M = 0;

if(B_R_M > 32000 ) B_R_M = 32000;
else if(B_R_M < 0) B_R_M = 0;

if(B_L_M > 32000 ) B_L_M = 32000;
else if(B_L_M < 0) B_L_M = 0;





//************////////// write to motors ////////////*****************
///////////////////////////////////////////////////////////////////////

if(M) {
// write to motors 
F_R_MOTOR.writeMicroseconds(map(F_R_M,0,32000,1000,2000));
F_L_MOTOR.writeMicroseconds(map(F_L_M,0,32000,1000,2000));
B_R_MOTOR.writeMicroseconds(map(B_R_M,0,32000,1000,2000));
B_L_MOTOR.writeMicroseconds(map(B_L_M,0,32000,1000,2000));
}
else {
F_R_MOTOR.writeMicroseconds(1000);
F_L_MOTOR.writeMicroseconds(1000);
B_R_MOTOR.writeMicroseconds(1000);
B_L_MOTOR.writeMicroseconds(1000);
}

////////////////////////////////////////////////////////////////////////////

if(R_throttle < 1250 && R_pitch > 1850 && !M)  {
  M = true  ;
  
  for(byte i=0;i<5;i++){
    digitalWrite(13,HIGH);
    delay(200);
    digitalWrite(13,LOW);
    delay(100);
    }
    
}
else if(R_throttle < 1250 && R_pitch < 1150) M = false ;

if(R_throttle > 1850 && R_pitch > 1850 && !M) caliberate_imu();

///////////////////////////////////////////////////////////////////////////

serialMoniter();          //printing data in  serialmoniter ...

//************////////// LOOP TIMER ////////////*****************
///////////////////////////////////////////////////////////////////////

while(micros() - loop_timer < 1000000/F);
loop_timer = micros();

//////////////////////////////////////////////////////////////////////////

}// end loop 


//This is the interruption routine
//----------------------------------------------

ISR(PCINT2_vect){
//First we take the current count value in micro seconds using the micros() function
  
  current_count = micros();
  ///////////////////////////////////////Channel 1
  if(PIND & B00010000){                              //We make an AND with the pin state register, We verify if pin 4 is HIGH???
    if(roll_state == 0){                         //If the last state was 0, then we have a state change...
      roll_state = 1;                            //Store the current state into the last state for the next loop
      counter_1 = current_count;                     //Set counter_1 to current value.
    }
  }
  else if(roll_state == 1){                      //If pin 4 is LOW and the last state was HIGH then we have a state change      
    roll_state = 0;                              //Store the current state into the last state for the next loop
    R = current_count - counter_1;   //We make the time difference. Channel 1 is current_time - timer_1.
    if(R<2100) R_roll = R/**a + R_roll*(1-a)*/ ;
//    if(R_roll>2000) R_roll = 2000;
//    else if(R_roll<1000) R_roll = 1000;
  }



  ///////////////////////////////////////Channel 2
  if(PIND & B00100000 ){                             //pin D5                                              
    if(yaw_state == 0){        
      yaw_state = 1;        
      counter_2 = current_count;                                             
    }
  }
  else if(yaw_state == 1){                                           
    yaw_state = 0;                                                     
    Y = current_count - counter_2;   
    if(Y<2100) R_yaw = Y/*a + R_yaw*(1-a)*/; 
//    if(R_yaw>2000) R_yaw = 2000;
//    else if(R_yaw<1000) R_yaw = 1000;                     
  }

}

void THROTTLE(void){

    if(PIND & B00000100 ){                             //pin D2                                             
    if(throttle_state == 0){        
      throttle_state = 1;        
      counter_3 = micros();                                             
    }
  }
  else if(throttle_state == 1){                                           
    throttle_state = 0;                                                     
    T = micros() - counter_3; 
    if(T<2100) R_throttle = T/*a + R_throttle*(1-a)*/;    
//    if(R_throttle>2000) R_throttle = 2000;
//    else if(R_throttle<1000) R_throttle = 1000;                        
  }
  
}

void PITCH(void){

    if(PIND & B00001000){                             //pin D3                                              
    if(pitch_state == 0){        
      pitch_state = 1;        
      counter_4 = micros();                                             
    }
  }
  else if(pitch_state == 1){                                           
    pitch_state = 0;                                                     
    P = micros() - counter_4;       
    if(P<2100) R_pitch = P/*a + R_pitch*(1-a)*/; 
//    if(R_pitch>2000) R_pitch = 2000;
//    else if(R_pitch<1000) R_pitch = 1000;                     
  }
  
}
