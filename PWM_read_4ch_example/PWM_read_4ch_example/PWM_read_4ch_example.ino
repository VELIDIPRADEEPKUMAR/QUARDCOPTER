
//We create variables for the time width values of each PWM input signal
unsigned long counter_1, counter_2, counter_3, counter_4, current_count;

//We create 4 variables to stopre the previous value of the input signal (if LOW or HIGH)
byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state;

//To store the 1000us to 2000us value we create variables and store each channel
int input_YAW;      //In my case channel 4 of the receiver and pin D12 of arduino
int input_PITCH;    //In my case channel 2 of the receiver and pin D9 of arduino
int input_ROLL;     //In my case channel 1 of the receiver and pin D8 of arduino
int input_THROTTLE; //In my case channel 3 of the receiver and pin D10 of arduino


void setup() {
  /*
   * Port registers allow for lower-level and faster manipulation of the i/o pins of the microcontroller on an Arduino board. 
   * The chips used on the Arduino board (the ATmega8 and ATmega168) have three ports:
     -B (digital pin 8 to 13)
     -C (analog input pins)
     -D (digital pins 0 to 7)
   
  //All Arduino (Atmega) digital pins are inputs when you begin...
  */  
   
  PCICR  |= (1 << PCIE2  );    //enable PCMSK0 scan                                                 
  PCMSK2 |= (1 << PCINT19);  //Set pin D3 trigger an interrupt on state change.  roll
  PCMSK2 |= (1 << PCINT20);  //Set pin D4 trigger an interrupt on state change.  pitch                                         
  PCMSK2 |= (1 << PCINT21);  //Set pin D5 trigger an interrupt on state change.  throttel                                            
  PCMSK2 |= (1 << PCINT22);  //Set pin D6 trigger an interrupt on state change.  yaw
                                                 
                                               
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  pinMode(6,INPUT);
  
  //Start the serial in order to see the result on the monitor
  //Remember to select the same baud rate on the serial monitor
  Serial.begin(9600);  

}

void loop() {
  /*
   * Ok, so in the loop the only thing that we do in this example is to print
   * the received values on the Serial monitor. The PWM values are read in the ISR below.
   */
    Serial.print(input_YAW);
    Serial.print("  *  ");
    Serial.print(input_PITCH );
    Serial.print("  *  ");
    Serial.print(input_ROLL );
    Serial.print("  *  ");
    Serial.println(input_THROTTLE );
   
   
   
  
}




//This is the interruption routine
//----------------------------------------------

ISR(PCINT2_vect){
//First we take the current count value in micro seconds using the micros() function
  
  current_count = micros();
  ///////////////////////////////////////Channel 1
  if(PIND & B00001000){                              //We make an AND with the pin state register, We verify if pin 3 is HIGH???
    if(last_CH1_state == 0){                         //If the last state was 0, then we have a state change...
      last_CH1_state = 1;                            //Store the current state into the last state for the next loop
      counter_1 = current_count;                     //Set counter_1 to current value.
    }
  }
  else if(last_CH1_state == 1){                      //If pin 8 is LOW and the last state was HIGH then we have a state change      
    last_CH1_state = 0;                              //Store the current state into the last state for the next loop
    input_ROLL = current_count - counter_1;   //We make the time difference. Channel 1 is current_time - timer_1.
  }



  ///////////////////////////////////////Channel 2
  if(PIND & B00010000 ){                             //pin D4 -- B00000010                                              
    if(last_CH2_state == 0){        
      last_CH2_state = 1;        
      counter_2 = current_count;                                             
    }
  }
  else if(last_CH2_state == 1){                                           
    last_CH2_state = 0;                                                     
    input_PITCH = current_count - counter_2;                             
  }


  
  ///////////////////////////////////////Channel 3
  if(PIND & B00100000 ){                             //pin D5 - B00000100                                         
    if(last_CH3_state == 0){                                             
      last_CH3_state = 1;                                                  
      counter_3 = current_count;                                               
    }
  }
  else if(last_CH3_state == 1){                                             
    last_CH3_state = 0;                                                    
    input_THROTTLE = current_count - counter_3;                            

  }


  
  ///////////////////////////////////////Channel 4
  if(PIND & B01000000 ){                             //pin D6  -- B00010000                      
    if(last_CH4_state == 0){                                               
      last_CH4_state = 1;                                                   
      counter_4 = current_count;                                              
    }
  }
  else if(last_CH4_state == 1){                                             
    last_CH4_state = 0;                                                  
    input_YAW = current_count - counter_4;                            
  }


 
}
