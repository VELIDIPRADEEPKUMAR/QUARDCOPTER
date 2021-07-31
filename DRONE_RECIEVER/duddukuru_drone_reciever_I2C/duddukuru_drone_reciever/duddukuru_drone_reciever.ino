
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t My_radio_pipeIn = 0xE8E8F0F0E1LL;     //Remember that this code is the same as in the transmitter
RF24 radio(7,8);  //CSN and CE pins

// The sizeof this struct should not exceed 32 bytes
struct Received_data {
  byte throttle;
  byte yaw ;
  byte pitch ;
//  byte yaw ;
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
  received_data.pitch = 127;
  received_data.roll = 127;
  received_data.ptm1 = 0;
  received_data.ptm2 = 0;
  received_data.t1 = 0;
  received_data.t2 = 0;
  received_data.t3 = 0;
  received_data.t4 = 0;
  
 // PPM_width_Values();
}



/**************************************************/


void setup(){
/**************************************************/
 
 
  //Call the reset data function
  reset_received_Data();

  //Once again, begin and radio configuration
  radio.begin();
  radio.setAutoAck(false);
  //radio.setDataRate(RF24_250KBPS);  
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(1,My_radio_pipeIn);
  
  //We start the radio comunication
  radio.startListening();

  Wire.begin(101);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(recieveEvent);
  
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  pinMode(6,INPUT);
  pinMode(9,OUTPUT);
  pinMode(2,OUTPUT);
  //Serial.begin(9600);
  
}

/**************************************************/

unsigned long lastRecvTime = 0;

//We create the function that will read the data each certain time
void receive_the_data()
{
  if( radio.available() ) {
  radio.read(&received_data, sizeof(Received_data));
  lastRecvTime = millis(); //Here we receive the data
}
}

/**************************************************/




void loop()
{
  //Receive the radio data
    receive_the_data();

  //////////This small if will reset the data if signal is lost for 1 sec.
/////////////////////////////////////////////////////////////////////////
  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    // signal lost?
    reset_received_Data();
    //Go up and change the initial values if you want depending on
    //your aplications. Put 0 for throttle in case of drones so it won't
    //fly away
  }
    
  
#if 0
  Serial.print(received_data.throttle);
  Serial.print("     ***   ");
  Serial.print(received_data.yaw);
  Serial.print("     ***   ");
  Serial.print(received_data.pitch);
  Serial.print("    ***   ");
  Serial.print(received_data.roll);
  Serial.print("   ***    ");
  Serial.print(received_data.ptm1);
  Serial.print("   ***   ");
  Serial.print(received_data.ptm2);
  Serial.print("   ***    ");
  Serial.print(received_data.t1);
  Serial.print("   ***    ");
  Serial.print(received_data.t2);
  Serial.print("   ***    ");
 // Serial.print(digitalRead(7));
  //Serial.print("   ***    ");
 // Serial.print(digitalRead(8));
  Serial.print("   ***    ");
  Serial.print(received_data.t3);
  Serial.print("   ***   ");
  Serial.println(received_data.t4);
 #endif
}//Loop end


void recieveEvent(void) {
  while ( Wire.available()) { // loop through all but the last
  // C[i]  = Wire.read();
  // Serial.println(Wire.read());
   //i = i + 1;
  }
}

void requestEvent(void){
  byte D[4];
   D[0]=received_data.throttle;
   D[1]=received_data.yaw;
   D[2]=received_data.pitch;
   D[3]=received_data.roll;
  // D[4]=5;
  // D[5]=25;
   
  Wire.write(D,4);
}
