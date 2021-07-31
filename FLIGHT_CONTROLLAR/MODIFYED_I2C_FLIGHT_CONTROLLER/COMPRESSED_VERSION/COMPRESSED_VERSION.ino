

/* This is a flight controller of mofified version of duddukuru flight controller for a drone with imu mpu6050 and i2c reciever input ,
 * This can be a test version 2.a 
 * pradeep 
 */

#include <Wire.h>
#include <Kalman.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

#define F  1000  // constrain loop frequiency 
uint16_t loop_timer ;

String datastring = ""; 

//************////////// IMU DATA ////////////*****************
///////////////////////////////////////////////////////////////////////
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
 
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;  // temperature = tempRaw / 340.0 + 36.53;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data


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
double PITCH_ERROR,PITCH_ERROR_P;
int16_t PITCH_P,PITCH_I,PITCH_D ;
int16_t PITCH_PID;

double ROLL_ERROR,ROLL_ERROR_P;
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
  
Serial.begin(9600);

pinMode(13,OUTPUT);
pinMode(8 ,OUTPUT);
pinMode(9 ,OUTPUT);
pinMode(10,OUTPUT);
pinMode(11,OUTPUT);

F_R_MOTOR.attach(10);
F_L_MOTOR.attach(3 );
B_R_MOTOR.attach(11);
B_L_MOTOR.attach(9 );

//////////////////////// caliberate motors ////////////////////
F_R_MOTOR.writeMicroseconds(2000);
F_L_MOTOR.writeMicroseconds(2000);
B_R_MOTOR.writeMicroseconds(2000);
B_L_MOTOR.writeMicroseconds(2000);

delay(2);

F_R_MOTOR.writeMicroseconds(1000);
F_L_MOTOR.writeMicroseconds(1000);
B_R_MOTOR.writeMicroseconds(1000);
B_L_MOTOR.writeMicroseconds(1000);
///////////////////////////////////////////////////////////////

Wire.begin();  // setup I2C 
  
#if ARDUINO >= 157
  Wire.setClock(100000UL); // Set I2C frequency to 100kHz
#else
  TWBR = ((F_CPU / 100000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

inilisizeImu() ; // imu setup ....

SD.begin();

//**************///////////// data logging ///////////////******************

datastring = " data logger ";

File datafile = SD.open("datalog.txt",FILE_WRITE);

if(datafile){
  datafile.println(datastring);
  datafile.close();
}

////////////////////////////////////////////////////////////////////////////////

// activating buzzer 

     for(i=0;i<20;i++){
    digitalWrite(8,HIGH);
    delay(100);
    digitalWrite(8,LOW);
    delay(50);
    }
    
//    for(i=0;i<5;i++){
//    digitalWrite(8,HIGH);
//    delay(400);
//    digitalWrite(8,LOW);
//    delay(300);
//    }
//    
     digitalWrite(8,LOW);

     
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

updateImuData(); // get imu data kalAngleX , kalAngleX 
// kalAngleX is for roll 
//  is for pitch

CaliculatePid(); 

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


if(RECIEVER_INPUT[0] < 25 && RECIEVER_INPUT[2] > 230)  {
  M = true  ;
  
  for(i=0;i<5;i++){
    digitalWrite(8,HIGH);
    delay(200);
    digitalWrite(8,LOW);
    delay(100);
    }
    
}
else if(RECIEVER_INPUT[0] < 25 && RECIEVER_INPUT[2] < 25) M = false ;


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


serialMoniter();          //printing data in  serialmoniter ...
datalogger();           // log data into sd card 

//************////////// LOOP TIMER ////////////*****************
///////////////////////////////////////////////////////////////////////

while(micros() - loop_timer < 1000000/F);
loop_timer = millis();

//////////////////////////////////////////////////////////////////////////

}


void datalogger(void){

  datastring = "  ";
  datastring += String(kalAngleX);
  datastring += "  ";
  datastring += String(kalAngleY);

//**************///////////// data logging ///////////////******************

File datafile = SD.open("datalog.txt",FILE_WRITE);

delayMicroseconds(10);

if(datafile){
  datafile.println(datastring);
  datafile.close();
}

////////////////////////////////////////////////////////////////////////////////

}

//-----------------------------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print("i2cWrite failed: ");
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
//-----------------------------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------
void updateImuData(void) { 

   /* Update all the values */
 
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
  

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

//  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
//  gyroYangle += gyroYrate * dt;
  gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
    

}
//-----------------------------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------
void inilisizeImu(void){

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

 
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

  timer = micros();
}

//-----------------------------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------
void CaliculatePid() { 



//************////////// PID CONTROLLER FOR PITCH ////////////*****************
///////////////////////////////////////////////////////////////////////

PITCH_ERROR = map(RECIEVER_INPUT[2], 0,255,-30,30) - kalAngleY ; 

PITCH_P = PITCH_ERROR*700;
PITCH_I = PITCH_I + PITCH_ERROR*1; 
PITCH_D = (PITCH_ERROR_P - PITCH_ERROR)*100;
PITCH_PID = PITCH_P + PITCH_I + PITCH_D ; 
PITCH_ERROR_P = PITCH_ERROR;

if(PITCH_PID > 15000) {           // sacturate pid 
  PITCH_I = PITCH_I - PITCH_PID + 15000;  // arrust the inc in integral 
  PITCH_PID = 15000;
}
else if(PITCH_PID < -15000) {            // sacturate pid 
  PITCH_I = PITCH_I - PITCH_PID - 15000;  // arrust the inc in integral 
  PITCH_PID = -15000;
}

//////////////////////////////////////////////////////////////////////////////

//************////////// PID CONTROLLER FOR ROLL ////////////*****************
///////////////////////////////////////////////////////////////////////

ROLL_ERROR = map(RECIEVER_INPUT[3], 0,255,-30,30) - kalAngleX ; 

ROLL_P = ROLL_ERROR*700;
ROLL_I = ROLL_I + ROLL_ERROR*1; 
ROLL_D = (ROLL_ERROR_P - ROLL_ERROR)*100;
ROLL_PID = ROLL_P + ROLL_I + ROLL_D ; 

if(ROLL_PID > 15000) {           // sacturate pid 
  ROLL_I = ROLL_I - ROLL_PID + 15000;  // arrust the inc in integral 
  ROLL_PID = 15000;
}
else if(ROLL_PID < -15000) {            // sacturate pid 
  ROLL_I = ROLL_I - ROLL_PID - 15000;  // arrust the inc in integral 
  ROLL_PID = -15000;
}

///////////////////////////////////////////////////////////////////////////

//************////////// P CONTROLLER FOR YAW ////////////*****************
///////////////////////////////////////////////////////////////////////

YAW_P = gyroZ;


}

//-----------------------------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------
void serialMoniter(void) {

#if 0 
Serial.println("********////////// INPUT JYOSTRICK DATA ////////////*********");
Serial.print("THROTTLE = ");
Serial.print(map(RECIEVER_INPUT[0],0,255,1000,2000));
Serial.print(" , ");
Serial.print("YAW = ");
Serial.print(map(RECIEVER_INPUT[1],0,255,1000,2000));
Serial.print(" , ");
Serial.print("PITCH = ");
Serial.print(map(RECIEVER_INPUT[2],0,255,1000,2000));
Serial.print(" , ");
Serial.print("ROLL = ");
Serial.println(map(RECIEVER_INPUT[3],0,255,1000,2000));

Serial.println("//-----------------------------------------------------------------------------//");

Serial.println("********////////// PITCH PID PARAMETERS  ////////////*********");
Serial.print("SET PITCH = ");
Serial.print(map(RECIEVER_INPUT[2], 0,255,-30,30));
Serial.print(" , ");
Serial.print("kalAngleY = ");
Serial.print(kalAngleY);
Serial.print(" , ");
Serial.print("ERROR = ");
Serial.println(PITCH_ERROR);
Serial.print("PITCH_P = ");
Serial.print(PITCH_P);
Serial.print(" , ");
Serial.print("PITCH_I = ");
Serial.print(PITCH_I);
Serial.print(" , ");
Serial.print("PITCH_D = ");
Serial.println(PITCH_D);
Serial.print("PITCH_PID = ");
Serial.println(PITCH_PID);

Serial.println("//-----------------------------------------------------------------------------//");

Serial.println("********////////// ROLL PID PARAMETERS  ////////////*********");
Serial.print("SET ROLL = ");
Serial.print(map(RECIEVER_INPUT[3], 0,255,-30,30));
Serial.print(" , ");
Serial.print("kalAngleX = ");
Serial.print(kalAngleX);
Serial.print(" , ");
Serial.print("ERROR = ");
Serial.println(ROLL_ERROR);
Serial.print("ROLL_P = ");
Serial.print(ROLL_P);
Serial.print(" , ");
Serial.print("ROLL_I = ");
Serial.print(ROLL_I);
Serial.print(" , ");
Serial.print("ROLL_D = ");
Serial.println(ROLL_D);
Serial.print("ROLL_PID = ");
Serial.println(ROLL_PID);

Serial.println("//-----------------------------------------------------------------------------//");

Serial.println("********////////// MOTOR SPEEDS ////////////*********");
Serial.print("F_L_M = ");
Serial.print(map(F_L_M,0,32000,1000,2000));
Serial.print(" , ");
Serial.print("F_R_M = ");
Serial.print(map(F_R_M,0,32000,1000,2000));
Serial.print(" , ");
Serial.print("B_L_M = ");
Serial.print(map(B_L_M,0,32000,1000,2000));
Serial.print(" , ");
Serial.print("B_R_M = ");
Serial.println(map(B_R_M,0,32000,1000,2000));

Serial.println("///////////////////////////////////////////////////////////////////////////////////////////////////////////");

#endif 


#if 0
Serial.println("********////////// ANGLES  ////////////*********");
Serial.print("kalAngleY = ");
Serial.print(kalAngleY);
Serial.print(" , ");
Serial.print("kalAngleX = ");
Serial.println(kalAngleX);
#endif

#if 1
Serial.println("********////////// RECIEVER INPUTS  ////////////*********");
Serial.print("THROTTLE = ");
Serial.print(map(RECIEVER_INPUT[0],0,255,1000,2000));
Serial.print(" , ");
Serial.print("SET PITCH = ");
Serial.print(map(RECIEVER_INPUT[2], 0,255,-30,30));
Serial.print(" , ");
Serial.print("SET ROLL = ");
Serial.println(map(RECIEVER_INPUT[3], 0,255,-30,30));

#endif


}
