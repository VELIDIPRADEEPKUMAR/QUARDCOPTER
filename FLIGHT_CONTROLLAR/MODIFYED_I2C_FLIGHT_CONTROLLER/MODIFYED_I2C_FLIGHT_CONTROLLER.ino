/* This is a flight controller of mofified version of duddukuru flight controller for a drone with imu mpu6050 and i2c reciever input ,
 * This can be a test version 2.a 
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
uint8_t RECIEVER_INPUT[4];   // I2C input data from reciever
/*  0 -> THROTTLE 
 *  1 -> YAW 
 *  2 -> PITCH 
 *  3 -> ROLL
 */
uint8_t i ;

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
bool R = false;
/////////////////////// SERVO OBJECTS /////////////////////////////////
Servo F_R_MOTOR ;
Servo F_L_MOTOR ;
Servo B_R_MOTOR ;
Servo B_L_MOTOR ;

//***********************************************************************//

void setup() {
  
Serial.begin(115200);

pinMode(3,OUTPUT);
pinMode(5,OUTPUT);
pinMode(6,OUTPUT);
pinMode(9,OUTPUT);
pinMode(8,OUTPUT);

F_R_MOTOR.attach(3);
F_L_MOTOR.attach(9);
B_R_MOTOR.attach(5);
B_L_MOTOR.attach(6);


F_R_MOTOR.writeMicroseconds(1000);
F_L_MOTOR.writeMicroseconds(1000);
B_R_MOTOR.writeMicroseconds(1000);
B_L_MOTOR.writeMicroseconds(1000);

//
////////////////////////// caliberate motors ////////////////////
//F_R_MOTOR.writeMicroseconds(2000);
//F_L_MOTOR.writeMicroseconds(2000);
//B_R_MOTOR.writeMicroseconds(2000);
//B_L_MOTOR.writeMicroseconds(2000);

delay(2);

///////////////////////////////////////////////////////////////

Wire.begin();  // setup I2C 
  
//#if ARDUINO >= 157
//  Wire.setClock(100000UL); // Set I2C frequency to 100kHz
//#else
//  TWBR = ((F_CPU / 100000UL) - 16) / 2; // Set I2C frequency to 400kHz
//#endif
//
//delay(2);

inilisizeImu() ; // imu setup ....


////////////////////////////////////////////////////////////////////////////////

// activating buzzer 

     for(i=0;i<20;i++){
    digitalWrite(8,HIGH);
    delay(100);
    digitalWrite(8,LOW);
    delay(50);
    }
    
     digitalWrite(8,LOW);

     imutimer = millis();
}// end setup 

void loop() {

//*********////////////// data from reciever ////////////***********
////////////////////////////////////////////////////////////////////
i = 0;
Wire.requestFrom(101,4);
while(Wire.available()){
  RECIEVER_INPUT[i] = Wire.read();
  i = i + 1;
}
//////////////////////////////////////////////////////////////////

updateImuData(); 



if(M){
  CaliculatePid(); 
  R = true;
}
else if(R){
  reset_pid();
  R = false;
}


// caliculate motor speeds 
F_R_M = map(RECIEVER_INPUT[0],0,255,0,32000) - PITCH_PID - ROLL_PID - YAW_P;
F_L_M = map(RECIEVER_INPUT[0],0,255,0,32000) - PITCH_PID + ROLL_PID + YAW_P;
B_R_M = map(RECIEVER_INPUT[0],0,255,0,32000) + PITCH_PID - ROLL_PID + YAW_P;
B_L_M = map(RECIEVER_INPUT[0],0,255,0,32000) + PITCH_PID + ROLL_PID - YAW_P;

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

if(RECIEVER_INPUT[0] < 25 && RECIEVER_INPUT[2] > 230 && !M)  {
  M = true  ;
  
  for(i=0;i<5;i++){
    digitalWrite(8,HIGH);
    delay(200);
    digitalWrite(8,LOW);
    delay(100);
    }
    
}
else if(RECIEVER_INPUT[0] < 25 && RECIEVER_INPUT[2] < 25) M = false ;

if(RECIEVER_INPUT[0] > 230 && RECIEVER_INPUT[2] > 230 && !M) caliberate_imu();

///////////////////////////////////////////////////////////////////////////

serialMoniter();          //printing data in  serialmoniter ...

//************////////// LOOP TIMER ////////////*****************
///////////////////////////////////////////////////////////////////////

while(micros() - loop_timer < 1000000/F);
loop_timer = micros();

//////////////////////////////////////////////////////////////////////////

}// end loop 
