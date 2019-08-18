
#include <SoftwareSerial.h>
int data=0;
int count= 325;
int flag = 0;
int save = 0;
int i = 0;
int threePings = 0;
int tpCount = 0;
int actuator_move = 0;
SoftwareSerial mySerial(15, 14); // RX, TX, plug your control line into pin 8 and connect it to the RX pin on the JRK21v3

//sets the new target for the JRK21V3 controller, this uses pololu high resulution protocal
void Move(int x) {
  word target = x;  //only pass this ints, i tried doing math in this and the remainder error screwed something up
  mySerial.write(0xAA); //tells the controller we're starting to send it commands
  mySerial.write(0xB);   //This is the pololu device # you're connected too that is found in the config utility(converted to hex). I'm using #11 in this example
  mySerial.write(0x40 + (target & 0x1F)); //first half of the target, see the pololu jrk manual for more specifics
  mySerial.write((target >> 5) & 0x7F);   //second half of the target, " " " 
} 

void setup()
{
//killswitch setup
pinMode(3,INPUT);
pinMode(4,INPUT);
pinMode(5,INPUT);
pinMode(6,INPUT);
pinMode(7,INPUT);
pinMode(8,OUTPUT);
attachInterrupt(1,RF_VT,RISING);


//actuator setup
mySerial.begin(9600);
Serial.begin(9600);
//SoftwareSerial mySerial(15, 14); // RX, TX, plug your control line into pin 8 and connect it to the RX pin on the JRK21v3

}


void loop()
{
  
  
  count++;                // number of cycles the on board arduino
                          // has done since the last transmition
  save = flag;            // saves value of data (value the kill switch transmitter sent)
  delay(10);
  
  if(data == 2){          // when data is the "go" value (2)
    threePings++;
    if(threePings >= 3){  // check that 3 transmitions have been 
                          // recieved before restarting kart
      threePings = 3;     // reset value of three pings received to 3
      count = 0;          // reset the counter to zero
    }
  }
                          // it takes 60 cycles to receive one transmition
  if(count < 325){        // as long as the count is below 130 cycles
    i++;                  // accelerator speed value
    if(i >= 1500){
      i=1500;
      // change i value to control speed of kart
      // higher i means faster kart, cannot exceed ~4V
    }
    analogWrite(8,i/15);  // slowly speed up the accelerator
   /*****************************
    * this is where the code goes for the kart moving
    *****************************/
    if(actuator_move == 1){
      Move(0);
      Serial.println("setting actuator to zero position");
      actuator_move = 0;
    }
  /// Enter code for kart logic

/****************************************************************************
 * 
 * This is where the code will go for when the kart is moving forward
 * 
 ****************************************************************************/
//=================================================================================






  //FIXME
  //enter code here







//================================================================================
    /// end code for kart logic
  }//end kart moving forward
  
  else{                   // if two transmitions are missed
    digitalWrite(8,0);    // stop acceleration
    
 /****************************************************************************
  * 
  * This is where the code will go for when the kart is stopped
  * 
  ****************************************************************************/
 //===============================================================================






  

  //FIXME
  //enter code here
  



  
  
 //===============================================================================   
    if(actuator_move == 0){
      Move(128);
      Serial.println("extending actuator");
      actuator_move = 1;
    }
    i=0;                  // reset acceleration
    if(threePings > 2){
       threePings = 0;       // resets number of pings received  
    }
  }//end kart stop
  
  if(flag == save){       // checks to see if data is the same 
                          // number as before
    data = 0;             //resets data to 0
  }

  if(count >= 800){       // makes sure the counter doesn't rise
                          // to an unreasonable number
    count = 325;
    threePings = 0;       // resets number of pings received 
    
  }
}//end main loop



//**************************************************************************
void RF_VT() // interrupt service function
{
  data=(digitalRead(4)<<3)+(digitalRead(5)<<2)+(digitalRead(6)<<1)
                                            +(digitalRead(7)<<0);
  flag++;
  if(flag == 200)
  {
    flag = 0;
  }
  Serial.print("Data: ");
  Serial.print(data);
  Serial.print("\tThreePings: ");
  Serial.print(threePings);
  Serial.print("\tCount: ");
  Serial.println(count);
}



 


















