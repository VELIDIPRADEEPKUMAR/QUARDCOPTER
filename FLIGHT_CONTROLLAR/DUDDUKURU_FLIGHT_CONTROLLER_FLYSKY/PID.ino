
void CaliculatePid(void) { 

//************////////// PID CONTROLLER FOR PITCH ////////////*****************
///////////////////////////////////////////////////////////////////////
if(map(R_throttle,1208,1920,1000,2000) >1100){
PITCH_ERROR = map(R_pitch, 1120,1920,-60,60) - (pitch - pitchoffset); 

PITCH_P = PITCH_ERROR*400;
PITCH_I = PITCH_I + PITCH_ERROR*0.01; 
PITCH_D = (PITCH_ERROR_P - PITCH_ERROR)*0;
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

}
else reset_pid();

//////////////////////////////////////////////////////////////////////////////

//************////////// PID CONTROLLER FOR ROLL ////////////*****************
///////////////////////////////////////////////////////////////////////
if(map(R_throttle,1208,1920,1000,2000) >1100){
ROLL_ERROR = map(R_roll-12, 1000,2000,-60,60) - (roll - rolloffset); 

ROLL_P = ROLL_ERROR*400;
ROLL_I = ROLL_I + ROLL_ERROR*0.01; 
ROLL_D = (ROLL_ERROR_P - ROLL_ERROR)*0;
ROLL_PID = ROLL_P + ROLL_I + ROLL_D ; 


if(ROLL_PID > 15000) {           // sacturate pid 
  ROLL_I = ROLL_I - ROLL_PID + 15000;  // arrust the inc in integral 
  ROLL_PID = 15000;
}
else if(ROLL_PID < -15000) {            // sacturate pid 
  ROLL_I = ROLL_I - ROLL_PID - 15000;  // arrust the inc in integral 
  ROLL_PID = -15000;
}

}
else reset_pid();
///////////////////////////////////////////////////////////////////////////

//************////////// P CONTROLLER FOR YAW ////////////*****************
///////////////////////////////////////////////////////////////////////




}






void reset_pid(void){
  ROLL_P    = 0;
  ROLL_I    = 0;
  ROLL_D    = 0;
  ROLL_PID  = 0;
  PITCH_P   = 0;
  PITCH_I   = 0;
  PITCH_D   = 0;
  PITCH_PID = 0;
  YAW_P     = 0;
}
