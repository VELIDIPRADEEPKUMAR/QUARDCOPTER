
void CaliculatePid(void) { 

//************////////// PID CONTROLLER FOR PITCH ////////////*****************
///////////////////////////////////////////////////////////////////////

PITCH_ERROR = map(RECIEVER_INPUT[2], 0,255,-30,30) - (pitch - pitchoffset); 

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

//////////////////////////////////////////////////////////////////////////////

//************////////// PID CONTROLLER FOR ROLL ////////////*****************
///////////////////////////////////////////////////////////////////////

ROLL_ERROR = map(RECIEVER_INPUT[3], 0,255,-30,30) - (roll - rolloffset); 

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
