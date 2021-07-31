


#include <SPI.h>
#include <nRF24L01.h> 
#include <RF24.h>

const uint64_t my_radio_pipe = 0xE8E8F0F0E1LL; 


RF24 radio(9,10);  //Set CE and CSN pins


struct Data_to_be_sent {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte ptm1;
  byte ptm2;
  byte t1;
  byte t2;
  byte t3;
  byte t4;
};


Data_to_be_sent sent_data;



void setup()
{
  //Serial.begin(9600);
  radio.begin();
  radio.setAutoAck(false);
  //radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(my_radio_pipe);
  radio.stopListening();

  //Reset each channel value
  pinMode(3,INPUT);
    pinMode(4,INPUT);
      pinMode(5,INPUT);
        pinMode(6,INPUT);
        
  sent_data.throttle = 0;
  sent_data.pitch = 127;
  sent_data.roll = 127;
  sent_data.ptm1 = 0;
  sent_data.ptm2 = 0;
  sent_data.t1 = 0;
  sent_data.t2 = 0;
  sent_data.t3 = 0;
  sent_data.t4 = 0;
}

/**************************************************/

int mapJoystickValues(int val, int lower, int middle, int upper,int sub1,int sub2)
{
  if( val>= (middle-sub1) && val<=(middle+sub2))
  {
    val = middle;
  }
  
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
    
   return(val);
}

void loop()
{
  /*If your channel is reversed, just swap 0 to 255 by 255 to 0 below
  EXAMPLE:
  Normal:    data.ch1 = map( analogRead(A0), 0, 1024, 0, 255);
  Reversed:  data.ch1 = map( analogRead(A0), 0, 1024, 255, 0);  */
  if(analogRead(A2) > 920){
    sent_data.throttle = 0;
  }
  else{
    sent_data.throttle = map((analogRead(A2)),0,1020,255,0);
  }
  
  sent_data.yaw =  mapJoystickValues(analogRead(A3),189,250,304,25,20);
  sent_data.pitch = mapJoystickValues(analogRead(A7),0,500,1023,1,1);
  sent_data.roll = mapJoystickValues(analogRead(A6),0,510,1023,1,1);
  sent_data.ptm1 = map( analogRead(A0), 0, 1024, 0, 255);
  sent_data.ptm2 = map( analogRead(A1), 0, 1024, 0, 255);
  sent_data.t1 = digitalRead(3);
  sent_data.t2 = digitalRead(4);
  sent_data.t3 = digitalRead(5);   
  sent_data.t4 = digitalRead(6);
  # if 0
  Serial.print(sent_data.throttle);
  Serial.print("        ");
  Serial.print(sent_data.pitch);
  Serial.print("        ");
  Serial.println(sent_data.roll);
  #endif 
  radio.write(&sent_data, sizeof(Data_to_be_sent));
}
