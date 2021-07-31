int ptime,btime,s=0,p=0;
int tcounter = 1,rcounter=0,ccounter=0;
int ch[11];
void setup() {
  // put your setup code here, to run once:

 PCICR |= (1 << PCIE2);    //enable PCMSK0 scan                                                 
  PCMSK2 |= (1 << PCINT18);
  Serial.begin(9600);
  int tcounter = 1,rcounter=0,ccounter=0;
  int ptime,btime,s=0,p=0;
}

void loop() {
  // put your main code here, to run repeatedly:   
  Serial.print(ch[0]+400);
  Serial.print("          ");
  Serial.print(ch[1]+400);
  Serial.print("           ");
  Serial.print(ch[2]+400);
  Serial.print("            ");
  Serial.print(ch[3]+400);
  Serial.print("            ");
  Serial.print(ch[4]+400);
  Serial.print("             ");
  Serial.print(ch[5]+400);
  Serial.print("             ");
  Serial.print(ch[6]+400);
  Serial.print("          ");
  Serial.println(ch[7]+400);
  
  
}






ISR(PCINT2_vect){
  
   ptime=micros();

   if(rcounter==1)
  {
    if(PIND & B00000100)
    {
      btime=ptime;
     // Serial.println(".................40..............................");
    }
    else
    {
     
    
      ch[ccounter]=ptime-btime;
      ccounter++;
      //Serial.println("................cc................................");
    }
    
     if(ccounter>10)
     {
      ccounter=0;
      //Serial.println("..........00......................................");
     }
     
  }

  
  
  if(tcounter==1)
  {
   
    if(PIND & B00000100)
    {
      btime=ptime;
      p=1;
      //Serial.println("........1.......................................");
    }
    else
    {
      if(p==1)
      {
      s=ptime-btime;

      if(s>2100)
    {
      tcounter=0;
      rcounter=1;
      //btime=ptime;
      //Serial.println("...........3....................................");
    }
     // Serial.println(".........2......................................");
    }
    }
     
   
  }

  
  
}
