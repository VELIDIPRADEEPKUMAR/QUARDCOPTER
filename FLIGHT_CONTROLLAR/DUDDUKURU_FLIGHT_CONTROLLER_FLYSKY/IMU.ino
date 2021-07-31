void updateImuData(void) { 

   /* Update all the values */
   
  // Read normalized values 
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate Pitch & Roll
 accPitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
 accRoll =   (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = (pitch + norm.YAxis *((millis() - imutimer)/1000.0))*0.97 + accPitch*0.03 ;
  roll = (roll + norm.XAxis * ((millis() - imutimer)/1000.0))*0.97 + accRoll*0.03;
  yaw = (yaw + norm.ZAxis * ((millis() - imutimer)/1000.0));

//////////********** LOW PASS FILTER ***********/////////////////
            pitch = pitch*0.4 + pitch_p*0.6 ;
            roll  = roll*0.4  + roll_p*0.6  ;
 ///////*************************************/////////////////////

 pitch_p = pitch ;
 roll_p =  roll ;

  imutimer = millis();
  
}






void inilisizeImu(void){

while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }


  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);

 rolloffset  = EEPROM.read(1);
 pitchoffset = EEPROM.read(2);
 
}




void caliberate_imu(void){


    digitalWrite(13,HIGH);
    delay(400);
    digitalWrite(13,LOW);
    delay(300);
    digitalWrite(13,HIGH);
    delay(200);
    digitalWrite(13,LOW);
    delay(400);
  
rolloffset = roll ;
pitchoffset = pitch;

EEPROM.write(1,rolloffset);
EEPROM.write(2,pitchoffset);

}
