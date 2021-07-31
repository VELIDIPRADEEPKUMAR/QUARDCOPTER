
//We create variables for the time width values of each PWM input signal
unsigned long counter_1, counter_2, counter_3, counter_4, current_count;

//We create 4 variables to stopre the previous value of the input signal (if LOW or HIGH)
byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state;

//To store the 1000us to 2000us value we create variables and store each channel
int input_YAW;      //In my case channel 4 of the receiver and pin D12 of arduino
int input_PITCH;    //In my case channel 2 of the receiver and pin D9 of arduino
int input_ROLL;     //In my case channel 1 of the receiver and pin D8 of arduino
int input_THROTTLE; //In my case channel 3 of the receiver and pin D10 of arduino




////////////////////// PPM CONFIGURATION//////////////////////////
#define channel_number 3  //set the number of channels, in this case: 4 channels
#define sigPin 2          //set PPM signal output pin on the arduino
#define PPM_FrLen 27000   //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 400  //set the pulse length
//////////////////////////////////////////////////////////////////

int ppm[4];




  
 

void PPM_width_Values()
{ 
  //Here we map the received values from 1000 to 2000
  //and create the ppm signals for each channel
  ppm[0] = map(input_THROTTLE,   1132, 1724, 1000, 2000);   // 0
  ppm[1] = map(input_YAW,   1130, 1940, 1000, 2000);                                                       /* map(received_data.yaw,   0, 255, 1000, 2000);*/      // 1
  ppm[2] = map(input_PITCH,   1196, 1848, 1000, 2000);      // 2
  ppm[3] = map(input_ROLL,  1148 , 1912, 1000, 2000);       // 3
        
  }

/**************************************************/


void setup(){
/**************************************************/
 
  //Configure the interruption registers that will create the PPM signal
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

  PCICR  |= (1 << PCIE2  );    //enable PCMSK0 scan                                                 
  PCMSK2 |= (1 << PCINT19);  //Set pin D3 trigger an interrupt on state change.  roll
  PCMSK2 |= (1 << PCINT20);  //Set pin D4 trigger an interrupt on state change.  pitch                                         
  PCMSK2 |= (1 << PCINT21);  //Set pin D5 trigger an interrupt on state change.  throttel                                            
  PCMSK2 |= (1 << PCINT22);  //Set pin D6 trigger an interrupt on state change.  yaw 
                                                 
  pinMode(2,OUTPUT);                                           
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  pinMode(6,INPUT);
  
  //Start the serial in order to see the result on the monitor
  //Remember to select the same baud rate on the serial monitor
  Serial.begin(9600);  
  
}

/**************************************************/




void loop()
{
  
    PPM_width_Values();  
    
#if 1
Serial.print(ppm[0]);
Serial.print("  *  ");
Serial.print(ppm[1]);
Serial.print("  *  ");
Serial.print(ppm[2]);
Serial.print("  *  ");
Serial.println(ppm[3]);
#endif 

 }






//#error Delete this line befor you cahnge the value (clockMultiplier) below
#define clockMultiplier 2 // set this to 2 if you are using a 16MHz arduino, leave as 1 for an 8MHz arduino



//Interruption vector. here we create the PPM signal
ISR(TIMER1_COMPA_vect){
  static boolean state = true;

  TCNT1 = 0;

  if ( state ) {
    //end pulse
    PORTD = PORTD & ~B00000100; // turn pin 2 off. Could also use: digitalWrite(sigPin,0)
    OCR1A = PPM_PulseLen * clockMultiplier;
    state = false;
  }
  else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B00000100; // turn pin 2 on. Could also use: digitalWrite(sigPin,1)
    state = true;

    if(cur_chan_numb > channel_number) {
      cur_chan_numb = 0;
      calc_rest += PPM_PulseLen;
      OCR1A = (PPM_FrLen - calc_rest) * clockMultiplier;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * clockMultiplier;
      calc_rest += ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}



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
