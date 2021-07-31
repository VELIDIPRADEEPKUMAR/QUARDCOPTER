#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

#include <SPI.h>



////////////////////// PPM CONFIGURATION//////////////////////////
#define channel_number 3  //set the number of channels, in this case: 4 channels
#define sigPin 2          //set PPM signal output pin on the arduino
#define PPM_FrLen 27000   //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 400  //set the pulse length
//////////////////////////////////////////////////////////////////

int ppm[channel_number];
char datain;
const uint64_t MY_RADIO_PIPEIN = 0xE8E8F0F0E1LL;
RF24 radio(7,8);   // csn ,ce;

// The sizeof this struct should not exceed 32 bytes
struct Received_data {
  byte throttle;
  byte yaw ;
  byte pitch ;
  byte roll;
  byte ptm1;
  byte ptm2;
  byte t1;
  byte t2;
  byte t3;
  byte t4;
};

Received_data received_data;



void reset_received_Data() 
{
  // 'safe' values to use when NO radio input is detected
  received_data.throttle = 0;      //Throttle (channel 1) to 0
  received_data.yaw = 127;
  received_data.pitch = 255;
  received_data.roll = 127;
  received_data.ptm1 = 0;
  received_data.ptm2 = 0;
  received_data.t1 = 0;
  received_data.t2 = 0;
  received_data.t3 = 0;
  received_data.t4 = 0;
  
  PPM_width_Values();
}

void PPM_width_Values()
{ 
  //Here we map the received values from 1000 to 2000
  //and create the ppm signals for each channel
  ppm[0] = map(received_data.throttle,   0, 255, 1000, 2000);   // 0
  ppm[1] = 1500;                                                          /* map(received_data.yaw,   0, 255, 1000, 2000);*/      // 1
  ppm[2] = map(received_data.pitch,   0, 255, 1000, 2000); 
  ppm[3] = map(received_data.roll,   0, 255, 1000, 2000);       // 3// 2
 ppm[9] = map(received_data.ptm1,   0, 255, 1000, 2000);       // 9
  ppm[5] = map(received_data.ptm2,   0, 255, 1000, 2000);       // 5
  ppm[6] = map(received_data.t1,   0, 1, 1000, 2000);           // 6
   ppm[7] = map(received_data.t2,   0, 1, 1000, 2000);          // 7
    ppm[8] = map(received_data.t3,   0, 1, 1000, 2000);         // 8
     ppm[4] = map(received_data.t4,   0, 1, 1000, 2000);  
     
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

  //Call the reset data function
 // reset_received_Data();

  radio.begin();
  radio.setAutoAck(false);
//radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.openReadingPipe(0,MY_RADIO_PIPEIN);
  radio.startListening();
  /*pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  pinMode(6,INPUT);
  pinMode(9,OUTPUT);*/
  pinMode(2,OUTPUT);
  Serial.begin(9600);
  
}

/**************************************************/
unsigned long lastRecvTime=0;

void RECIEVE_DATA(){
  if(radio.available()){
    radio.read(&received_data,sizeof(Received_data));
    lastRecvTime = millis();
  }
}




void loop()
{
  
     RECIEVE_DATA();

  
     
unsigned long now = millis();
if((now - lastRecvTime)>1000){
   //reset_received_Data();
   received_data.throttle = 0;      //Throttle (channel 1) to 0
  received_data.yaw = 127;
  received_data.pitch = 255;
  received_data.roll = 127;
  received_data.ptm1 = 0;
  received_data.ptm2 = 0;
  received_data.t1 = 0;
  received_data.t2 = 0;
  received_data.t3 = 0;
  received_data.t4 = 0;
}
 

  
#if 1
  Serial.print(ppm[0]);
  Serial.print("     ***   ");
  Serial.print(ppm[1]);
  Serial.print("     ***   ");
  Serial.print(ppm[2]);
  Serial.print("    ***   ");
  Serial.print(ppm[3]);
  Serial.print("   ***    ");
  Serial.print(ppm[4]);
  Serial.print("   ***   ");
  Serial.print(ppm[5]);
  Serial.print("   ***    ");
  Serial.print(ppm[6]);
  Serial.print("   ***    ");
  Serial.print(ppm[7]);
  Serial.print("   ***    ");
 // Serial.print(digitalRead(7));
  //Serial.print("   ***    ");
 // Serial.print(digitalRead(8));
  Serial.print("   ***    ");
  Serial.print(ppm[8]);
  Serial.print("   ***   ");
  Serial.println(ppm[9]);
 #endif
}//Loop end






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
