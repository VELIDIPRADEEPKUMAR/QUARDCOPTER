void UPDATE_PID_VARIABLES(){

//////////////ROLL PID SETPOINT ////////////////

  if(input_ROLL < 1480){
    ROLL_PID_SETPOINT = map(input_ROLL,1000,1490,-30,0);
  }
  else if(input_ROLL > 1520){
    ROLL_PID_SETPOINT = map(input_ROLL,1510,2000,0,30);
  }
 else {
    ROLL_PID_SETPOINT = 0;
 }
 



//////////////PITCH PID SETPOINT /////////////////////

 if(input_PITCH < 1490){
  PITCH_PID_SETPOINT = map(input_PITCH,1000,1490,-30,0);
 }
 else if(input_PITCH > 1510){
  PITCH_PID_SETPOINT = map(input_PITCH,1510,2000,0,30);
 }
 else{
    PITCH_PID_SETPOINT = 0;
 }

///////////// ROLL PID INPUT /////////////////////////

ROLL_PID_INPUT = fangleroll;

//////////// PITCH PID INPUT ////////////////////////

PITCH_PID_INPUT = fanglepitch;


}



void CALCULATE_PID(){

PID_ROLL.Compute();
PID_PITCH.Compute();



  
}
