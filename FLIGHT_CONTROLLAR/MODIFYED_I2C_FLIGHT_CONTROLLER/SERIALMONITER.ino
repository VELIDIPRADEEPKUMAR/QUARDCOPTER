
void serialMoniter(void) {
#if 1

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
#endif 

#if 0
Serial.println("********////////// PITCH PID PARAMETERS  ////////////*********");
Serial.print("SET PITCH = ");
Serial.print(map(RECIEVER_INPUT[2], 0,255,-30,30));
Serial.print(" , ");
Serial.print("kalAngleY = ");
Serial.print(pitch);
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
#endif 

#if 0 
Serial.println("********////////// ROLL PID PARAMETERS  ////////////*********");
Serial.print("SET ROLL = ");
Serial.print(map(RECIEVER_INPUT[3], 0,255,-30,30));
Serial.print(" , ");
Serial.print("kalAngleX = ");
Serial.print(roll);
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
#endif 

#if 0 
//Serial.println("********////////// MOTOR SPEEDS ////////////*********");
//Serial.print("F_L_M = ");
Serial.print(map(F_L_M,0,32000,1000,2000));
Serial.print(" , ");
//Serial.print("F_R_M = ");
Serial.print(map(F_R_M,0,32000,1000,2000));
Serial.print(" , ");
//Serial.print("B_L_M = ");
Serial.print(map(B_L_M,0,32000,1000,2000));
Serial.print(" , ");
//Serial.print("B_R_M = ");
Serial.println(map(B_R_M,0,32000,1000,2000));

//Serial.println("///////////////////////////////////////////////////////////////////////////////////////////////////////////");
#endif 


#if 1
//Serial.println("********////////// ANGLES  ////////////*********");
Serial.print(" pitch = ");
Serial.println(pitch - pitchoffset);  
Serial.print(" roll= ");
Serial.print(roll - rolloffset);
//Serial.print(" yaw= ");
//Serial.println(yaw);

#endif

#endif


}
