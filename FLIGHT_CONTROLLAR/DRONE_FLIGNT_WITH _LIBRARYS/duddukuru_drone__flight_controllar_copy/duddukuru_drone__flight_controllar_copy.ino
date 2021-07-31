
 /* 
  *  this code is simple flight controllar code with ppm compatable 
  *  this is primary code of flight controllar 
  */
 
//Includes
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <Kalman.h>

Servo L_F_prop;     
Servo L_B_prop;
Servo R_F_prop;
Servo R_B_prop;
int p;
int offsetx,offsety;
///////////////////////////////  KALMAN /////////////////////////////
 // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH   // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
 
/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

////////////////////////////////////////////////////////////



//////////  LOW PASS FILTER //////////


float fx,fy;
#define FREQ     100
float err,erp;
int j=0;
float pitcherror,rollerror;
float pidpitchprev,pidrollprev;


///////////////////////  ppm read data //////////////////
int ptime,btime,s;
int tcounter = 1,rcounter=0,ccounter=0;
int ch[11];
#define MPU_ADDRESS 0x68
int  input_ROLL =0 ;
 int input_PITCH =0;
  int input_THROTTLE=0;
  int input_YAW =0;

    float temp;
int i=0;
  // vibracation 
  int esc=0;

  float vibp=0;
  float vibr=0;
 float fangleroll,fanglepitch; 
 



unsigned int  period; // Sampling   
unsigned long loop_timer;
unsigned long now, difference;
float esclooptimer;
double start_motor= -2000;
float elapsedTime, timet, timePrev,time; 



//More variables for the code

int mot_activated = 0;
long activate_count=0;
long des_activate_count=0;

////////////// pid esc value //////////////
int  pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B;


//////////////// PID ROLL //////////////////////////////// 
double ROLL_PID_INPUT ;
double ROLL_PID_OUTPUT;
double ROLL_PID_SETPOINT;


///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp=20;//3.55//.9
double roll_ki=.0;//0.003//0.006
double roll_kd=.5;//2.05/1.2


//////////////// PID PITCH //////////////////////////////// 
double PITCH_PID_INPUT ;
double PITCH_PID_OUTPUT;
double PITCH_PID_SETPOINT;

///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp=roll_kp;//3.55
double pitch_ki=roll_ki;//0.003
double pitch_kd=roll_kd;//2.05

int addr=0;

PID PID_ROLL(&ROLL_PID_INPUT,&ROLL_PID_OUTPUT,&ROLL_PID_SETPOINT,roll_kp,roll_ki,roll_kd,DIRECT);
PID PID_PITCH(& PITCH_PID_INPUT ,&PITCH_PID_OUTPUT, &PITCH_PID_SETPOINT,pitch_kp,pitch_ki,pitch_kd,DIRECT);


void setup() {

pinMode(13,OUTPUT);
pinMode(8,OUTPUT);
pinMode(9,OUTPUT);
pinMode(10,OUTPUT);
pinMode(11,OUTPUT);
//PORTB |= B00100000; //D13 high 
//digitalWrite(13,HIGH);
 PCICR |= (1 << PCIE2);    //enable PCMSK0 scan                                                 
  PCMSK2 |= (1 << PCINT18); // digital pin 2 as interupt 

  L_F_prop.attach(8); //left front motor
  L_B_prop.attach(9); //left back motor
  R_F_prop.attach(10); //right front motor 
  R_B_prop.attach(11); //right back motor 
  /*in order to make sure that the ESCs won't enter into config mode
  *I send a 1000us pulse to each ESC.*/
  L_F_prop.writeMicroseconds(1000); 
  L_B_prop.writeMicroseconds(1000);
  R_F_prop.writeMicroseconds(1000); 
  R_B_prop.writeMicroseconds(1000);
  
  //PORTB |= B11000000;
  //PORTB &= B11000000;
  PID_ROLL.SetMode(AUTOMATIC);
  PID_PITCH.SetMode(AUTOMATIC);
  PID_ROLL.SetOutputLimits(-400,400);
  PID_PITCH.SetOutputLimits(-400,400);
  PID_ROLL.SetSampleTime(10);
  PID_PITCH.SetSampleTime(10);
  

//Serial.begin(9600); 
Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
 period = (1000000/FREQ) ; // Sampling period in µs

    // Initialize loop_timer
    loop_timer = micros();
    
    for(i=0;i<200;i++){
    digitalWrite(13,HIGH);
    delay(6);
    digitalWrite(13,LOW);
    delay(3);
    
    }
     digitalWrite(13,LOW);


    p=0;
}//end of setup void






void loop() {
    
   input_ROLL = ch[3] + 400 ;
  input_PITCH = ch[2] + 400 ;
  input_THROTTLE = ch[0] + 400 ;
  
 

  timePrev = time;                        // the previous time is stored before the actual time read
  time = millis();                        // actual time read
  elapsedTime = (time - timePrev) / 1000; //divide by 1000 in order to obtain seconds


/////////////////// READ THE IMU DATA ////////////////  
  // imu with kalman fulter 
  
   IMU_READ();
   
///////////////////////////////////////////////////////

 fanglepitch =  (-1*kalAngleX) ;//- pitchviberror;     //  (B -)(F +)   (.1*fanglepitch )+(.9*Total_angle_y);
 fangleroll  =  (kalAngleY ) ;//- rollviberror; ;      //   (R +)(L -)  (.1*fangleroll)+(.9*Total_angle_x);

/////////////////////////// calculate pid//////////

 UPDATE_PID_VARIABLES();
 CALCULATE_PID();

////////////////////////////////////////////////






/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwm_R_F  =   input_THROTTLE - ROLL_PID_OUTPUT - PITCH_PID_OUTPUT;
pwm_R_B  =   input_THROTTLE - ROLL_PID_OUTPUT + PITCH_PID_OUTPUT;
pwm_L_B  =   input_THROTTLE + ROLL_PID_OUTPUT + PITCH_PID_OUTPUT;
pwm_L_F  =   input_THROTTLE + ROLL_PID_OUTPUT - PITCH_PID_OUTPUT;
      
 // Refresh rate is 41.666Hz: send ESC pulses every 24000µs

 
   
/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right front
if(pwm_R_F < 1000)
{
  pwm_R_F= 1000;
}
if(pwm_R_F > 2000)
{
  pwm_R_F=2000;
}

//Left front
if(pwm_L_F < 1000)
{
  pwm_L_F= 1000;
}
if(pwm_L_F > 2000)
{
  pwm_L_F=2000;
}

//Right back
if(pwm_R_B < 1000)
{
  pwm_R_B= 1000;
}
if(pwm_R_B > 2000)
{
  pwm_R_B=2000;
}

//Left back  
if(pwm_L_B < 1000)
{
  pwm_L_B= 1000;
}
if(pwm_L_B > 2000)
{
  pwm_L_B=2000;
}





#if 0
//Serial.println(esc);
 //Serial.print("|");
 //Serial.print(pitch_desired_angle);
 //Serial.print("|");*/
 
//Serial.print(pitch_PID);
//Serial.print("  |  ");
//Serial.println(roll_PID);
//Serial.print("     |    ");
 
Serial.print(input_THROTTLE);
Serial.print(",");
 Serial.print(input_ROLL );
 Serial.print(",");
Serial.print(input_PITCH );
 Serial.print(",");

 
 //Serial.print("RF: ");
 Serial.print(pwm_R_F);
 Serial.print(",");
 //Serial.print("RB: ");
 Serial.print(pwm_R_B);
 Serial.print(",");
 //Serial.print("LB: ");
 Serial.print(pwm_L_B);
 Serial.print(",");
 //Serial.print("LF: ");
 Serial.print(pwm_L_F);


 //Serial.print("   |  ");
 //Serial.println(start_motor);
 Serial.print("  ,  ");
 //Serial.print("Xº: ");
Serial.print( fangleroll);
 Serial.print("  ,  ");
//Serial.print("Yº: ");
 Serial.println( fanglepitch);
// Serial.print("  |  ");
// Serial.println(elapsedTime);
 
#endif
  







/*now we can write the values PWM to the ESCs only if the motor is activated
*/

  //start_motor=2000; 
  if((input_THROTTLE < 1100) && (input_PITCH < 1100) && (start_motor<0) && (input_THROTTLE > 900 ))
  {
   
      start_motor=2000;

         pwm_R_F = 1000;
         pwm_L_B = 1000;
         pwm_R_F = 1000;
         pwm_R_B = 1000;
     for(i=0;i<20;i++){
    digitalWrite(13,HIGH);
    delay(100);
    digitalWrite(13,LOW);
    delay(50);
    
    }
      
  }  
  

   if((input_THROTTLE < 1100) && (input_PITCH > 1800) && (start_motor>0))
  {
      start_motor=-2000;
      digitalWrite(13,HIGH);
  }


 

  
 ////////////////////////////////// loop break ////////////////////
  
  while ((( micros()) - loop_timer) < (1000000/FREQ));
     loop_timer = micros(); 
/////////////////*****************************////////////////////////////

//*****************///////WRITE MOTORS////////****************************




      if(start_motor>0)
     {
       //PORTB=PORTB|B00001111;    //WRITING  PIN 8,9,10,11 HIGH
       //int loop_timer1 = micros();
      
        L_F_prop.writeMicroseconds(pwm_L_F);    // 8   lf[0] = pwm_L_F;
        L_B_prop.writeMicroseconds(pwm_L_B);    // 9   lb[0] = pwm_L_B;
        R_F_prop.writeMicroseconds(pwm_R_F);    // 10  rf[0] = pwm_R_F;
        R_B_prop.writeMicroseconds(pwm_R_B);    // 11  fb[0] = pwm_R_B;
  
 
  
  }
 

 if(start_motor<0)
  {
  L_F_prop.writeMicroseconds(1000); 
  L_B_prop.writeMicroseconds(1000);
  R_F_prop.writeMicroseconds(1000); 
  R_B_prop.writeMicroseconds(1000);
 
  }
//////////////////////////////////////////////////////////////////     
   
  
 


}




ISR(PCINT2_vect){
  
   ptime=micros();
  // ptime = TIMER2_BASE->CCR1;

   if(rcounter==1)
    {
    if(PIND & B00000100)
    {
      btime=ptime;
     // Serial.println(".................40..............................");
    }
    else
    {
    // if(ptime-btime < 0)ptime += 0xFFFF ;
    
      ch[ccounter]=ptime-btime;
      ccounter++;
      //Serial.println("................cc................................");
    }
    
     if(ccounter>4)
     {
      ccounter=0;
      //Serial.println("..........00......................................");
     }
     
  }

  
  
  if(tcounter==1)
  {
   
    if(PIND & B00000100)
    {
      btime=ptime;
      p=1;
      //Serial.println("........1.......................................");
    }
    else
    {
      if(p==1)
      {
      s=ptime-btime;

      if(s>2100)
    {
      tcounter=0;
      rcounter=1;
      //btime=ptime;
      //Serial.println("...........3....................................");
    }
     // Serial.println(".........2......................................");
    }
    }
     
   
  }

  
  
}
