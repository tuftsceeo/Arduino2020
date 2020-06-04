
/* Connect to Grove Loadcell Server for MATLAB    */
/* modified from Giampiero Campa, Copyright 2009 The MathWorks, Inc       */

/*Modified 1/3/2012 by R White to include an interrupt on pin 2 which
reads quadrature encoder, a single pass low pass filter on this data to
get velocity estimate (break frequency can be set as a constant below)
and two new states which allow you to query for position or velocity over
the serial port (state 50) or drive either motor (using Ardumoto pins) (state 60)*/

/*Modified 6/2/2020 by R White to include communications with the Grove
 * load cell using the HX711 libraries (state 70)
 */

/* This file is meant to be used with the MATLAB arduino IO 
   package, however, it can be used from the IDE environment
   (or any other serial terminal) by typing commands like:
   
   0e0 : assigns digital pin #4 (e) as input
   0f1 : assigns digital pin #5 (f) as output
   0n1 : assigns digital pin #13 (n) as output   
   
   1c  : reads digital pin #2 (c) 
   1e  : reads digital pin #4 (e) 
   2n0 : sets digital pin #13 (n) low
   2n1 : sets digital pin #13 (n) high
   2f1 : sets digital pin #5 (f) high
   2f0 : sets digital pin #5 (f) low
   4j2 : sets PWM pin #9 (j) to  50=ascii(2) over 255
   4jz : sets PWM pin #9 (j) to 122=ascii(z) over 255
   3a  : reads analog pin #0 (a) 
   3f  : reads analog pin #5 (f) 

   5a : reads position
   5b : reads filtered velocity
   5c : reads raw velocity

   6a2 : drives motor A forward at 50 (ascii 2) 
   6bz : drives motor B forward at 122 (ascii z)
   6cz : drives motor A backward at 122 (ascii z)
   6d2 : drives motor B backward at 50 (ascii 2)
    
   R0  : sets analog reference to DEFAULT
   R1  : sets analog reference to INTERNAL
   R2  : sets analog reference to EXTERNAL
  
   99  : returns script type (1 basic, 2 motor, 3 general)   */

//Loadcell
#include "HX711.h" //library for reading HX711 loadcell
HX711 loadcell;

// 1. HX711 circuit wiring (CHECK THIS!)
const int LOADCELL_DOUT_PIN = 6;
const int LOADCELL_SCK_PIN = 5;

// 2. Adjustment settings
//const long LOADCELL_OFFSET = 50682624;
//const long LOADCELL_DIVIDER = 5895655;

//Motor 1:
#define APin  2
#define BPin  4
#define DriveA 3
#define DirA 12

//Motor 2: //Check this on ARDUMOTO:
#define DriveB 11
#define DirB 13


//Some Variables for Quadrature Encoder velocity estimate
//Motor 1:
volatile float rawvel = 0; //motor 1 direct velocity measurement in degrees/sec
long int lastencoderedge = 0;//last time the encoder edge moved (microseconds)
float vel=0; //velocity state (with low pass filter)
float vel_old=0; //old velocity state (with low pass filter)
float velbreak=10; //frequency for low pass filter (Hz)
unsigned long time = 0; //current time
unsigned long oldtime =0; //old time
float dt =0; //delta time

//Some Variables for Quadrature Encoder displacement estimate
volatile long int position1 = 0; //motor 1 position in degrees

//Some variables for motor driving
int PWMpin=3;
int DIRpin=12;

  /* variables declaration and initialization                */
  
  static int  s   = -1;    /* state                          */
  static int  pin = 13;    /* generic pin number             */

  int  val =  0;           /* generic value read from serial */
  int  agv =  0;           /* generic analog value           */
  int  dgv =  0;           /* generic digital value          */


void setup() {
  /* Make sure all pins are put in high impedence state and 
     that their registers are set as low before doing anything.
     This puts the board in a known (and harmless) state     */
  int i;
  for (i=0;i<20;i++) {
    pinMode(i,INPUT);
    digitalWrite(i,0);
  }
  /* initialize serial                                       */
  Serial.begin(115200);
  
  //Set up quad encoder input pins:
  pinMode(APin,INPUT);
  pinMode(BPin,INPUT);
  
  //Set up motor drive pins:
  pinMode(DriveA,OUTPUT);
  pinMode(DirA,OUTPUT);
  pinMode(DriveB,OUTPUT);
  pinMode(DirB,OUTPUT);
  
  //Create hardware interrupt on pin 2 for motor 1 quad encoder
  attachInterrupt(0, doEncoder1, CHANGE); //Motor 1 on Pin 2 interrupt

  // 3. Initialize HX711 loadcell library
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  loadcell.set_scale();
  loadcell.tare();

}


void loop() {
    
  // record the time and sensor data:
  oldtime=time;
  time=millis(); //note time is in milliseconds
  dt=(time-oldtime);
  
  //One pole low pass filter on velocity (note dt in milliseconds):
  vel_old=vel;
  vel=(1000*vel_old+(velbreak*6.28)*rawvel*dt)/(velbreak*6.28*dt+1000);
  
  //If the motor is not turning at all, we will never get an update to
  //rawvel since this happens in the interrupt.  So, here is a little
  //thing to just check if lastencoderedge happened too long ago then
  //velocity must really be zero.  Assumes minimum velocity before motor
  //sticks is 30 degrees/second
  if ((micros()-lastencoderedge)>33000)
    rawvel=0;
    
    /* The following instruction constantly checks if anything 
     is available on the serial port. Nothing gets executed in
     the loop if nothing is available to be read, but as soon 
     as anything becomes available, then the part coded after 
     the if statement (that is the real stuff) gets executed */

  
  if (Serial.available() >0) {

    /* whatever is available from the serial is read here    */
    val = Serial.read();
    
    /* This part basically implements a state machine that 
       reads the serial port and makes just one transition 
       to a new state, depending on both the previous state 
       and the command that is read from the serial port. 
       Some commands need additional inputs from the serial 
       port, so they need 2 or 3 state transitions (each one
       happening as soon as anything new is available from 
       the serial port) to be fully executed. After a command 
       is fully executed the state returns to its initial 
       value s=-1                                            */

    switch (s) {

		
      /* s=-1 means NOTHING RECEIVED YET ******************* */
      case -1:      

      /* calculate next state                                */
      if (val>47 && val<90) {
	  /* the first received value indicates the mode       
           49 is ascii for 1, ... 90 is ascii for Z          
           s=0 is change-pin mode
           s=10 is DI;  s=20 is DO;  s=30 is AI;  s=40 is AO; 
           s=50 is encoder read; s=60 is motor drive
           s=70 is loadcell read
           s=90 is query script type (1 basic, 2 motor)        
           s=340 is change analog reference         
                                                             */
        s=10*(val-48);
      }
      
      /* the following statements are needed to handle 
         unexpected first values coming from the serial (if 
         the value is unrecognized then it defaults to s=-1) */
      if ((s>71 && s<90) || (s>90 && s!=340)) {
        s=-1;
      }

      /* the break statements gets out of the switch-case, so
      /* we go back to line 97 and wait for new serial data  */
      break; /* s=-1 (initial state) taken care of           */


     
      /* s=0 or 1 means CHANGE PIN MODE                      */
      
      case 0:
      /* the second received value indicates the pin 
         from abs('c')=99, pin 2, to abs('t')=116, pin 19    */
      if (val>98 && val<117) {
        pin=val-97;                /* calculate pin          */
        s=1; /* next we will need to get 0 or 1 from serial  */
      } 
      else {
        s=-1; /* if value is not a pin then return to -1     */
      }
      break; /* s=0 taken care of                            */


      case 1:
      /* the third received value indicates the value 0 or 1 */
      if (val>47 && val<50) {
        /* set pin mode                                      */
        if (val==48) {
          pinMode(pin,INPUT);
        }
        else {
          pinMode(pin,OUTPUT);
        }
      }
      s=-1;  /* we are done with CHANGE PIN so go to -1      */
      break; /* s=1 taken care of                            */
      


      /* s=10 means DIGITAL INPUT ************************** */
      
      case 10:
      /* the second received value indicates the pin 
         from abs('c')=99, pin 2, to abs('t')=116, pin 19    */
      if (val>98 && val<117) {
        pin=val-97;                /* calculate pin          */
        dgv=digitalRead(pin);      /* perform Digital Input  */
        Serial.println(dgv);       /* send value via serial  */
      }
      s=-1;  /* we are done with DI so next state is -1      */
      break; /* s=10 taken care of                           */

      

      /* s=20 or 21 means DIGITAL OUTPUT ******************* */
      
      case 20:
      /* the second received value indicates the pin 
         from abs('c')=99, pin 2, to abs('t')=116, pin 19    */
      if (val>98 && val<117) {
        pin=val-97;                /* calculate pin          */
        s=21; /* next we will need to get 0 or 1 from serial */
      } 
      else {
        s=-1; /* if value is not a pin then return to -1     */
      }
      break; /* s=20 taken care of                           */

      case 21:
      /* the third received value indicates the value 0 or 1 */
      if (val>47 && val<50) {
        dgv=val-48;                /* calculate value        */
	digitalWrite(pin,dgv);     /* perform Digital Output */
      }
      s=-1;  /* we are done with DO so next state is -1      */
      break; /* s=21 taken care of                           */


	
      /* s=30 means ANALOG INPUT *************************** */
      
      case 30:
      /* the second received value indicates the pin 
         from abs('a')=97, pin 0, to abs('f')=102, pin 6,     
         note that these are the digital pins from 14 to 19  
         located in the lower right part of the board        */
      if (val>96 && val<103) {
        pin=val-97;                /* calculate pin          */
        agv=analogRead(pin);       /* perform Analog Input   */
	      Serial.println(agv);       /* send value via serial  */
      }
      s=-1;  /* we are done with AI so next state is -1      */
      break; /* s=30 taken care of                           */
	


      /* s=40 or 41 means ANALOG OUTPUT ******************** */
      
      case 40:
      /* the second received value indicates the pin 
         from abs('c')=99, pin 2, to abs('t')=116, pin 19    */
      if (val>98 && val<117) {
        pin=val-97;                /* calculate pin          */
        s=41; /* next we will need to get value from serial  */
      }
      else {
        s=-1; /* if value is not a pin then return to -1     */
      }
      break; /* s=40 taken care of                           */


      case 41:
      /* the third received value indicates the analog value */
      analogWrite(pin,val);        /* perform Analog Output  */
      s=-1;  /* we are done with AO so next state is -1      */
      break; /* s=41 taken care of                           */
      
      /* I am adding a case to take care of quadrature encoder
      data, and driving two motors using ARDUMOTO.  - R White 1/3/2012 */

      /*******CASE 50 is reading position/velocity*****/
      
      case 50:
      /* the second received value indicates whether motor position
      'a' or motor filtered velocity 'b' or motor raw velocity 'c' is desired.  
      Result is returned in
      degrees (position) or degrees per second (velocity)
      option 'd' zeros the encoder and velocity state
      */
      if (val==97) //motor position
      {
        Serial.println(position1);       /* send value via serial  */
      }
      else if (val==98) //motor velocity
      {
        Serial.println(int(vel));
      }
      else if (val==99) //motor velocity
      {
        Serial.println(int(rawvel));
      }
      else if (val==100) //zero states
      {
        vel=0;
        position1=0;
        Serial.println(int(position1));
      }

      s=-1;  /* we are done with quad reading so next state is -1      */
      break; /* s=50 taken care of                           */


      /*******CASE 60 is driving motor*****/
      
      case 60:
      /* the second received value indicates whether motor a forward
      'a' or motor b forward 'b', or motor a backward 'c' or motor b backward
      'd' is desired.  The third received value gives a drive 
      level where 255 is max forward, 0 is off
      */
      if (val==97) //drive motor A forward
      {
        PWMpin=DriveA;
        digitalWrite(DirA,HIGH);
        s=61;  /* we know which motor and direction - get one more value for drive level      */
      }
      else if (val==98) //drive motor B forward
      {
        PWMpin=DriveB;
        digitalWrite(DirB,HIGH);
        s=61;  /* we know which motor - get one more value for drive level      */
      }
      else if (val==99) //drive motor A backward
      {
        PWMpin=DriveA;
        digitalWrite(DirA,LOW);
        s=61;  /* we know which motor - get one more value for drive level      */
      }
      else if (val==100) //drive motor B backward
      {
        PWMpin=DriveB;
        digitalWrite(DirB,LOW);
        s=61;  /* we know which motor - get one more value for drive level      */
      }
      else
      {
        s=-1;
      }
      break; /* s=60 taken care of                           */

      case 61:
      if (val>=0 && val<=255) {
        analogWrite(PWMpin,val);
      }
      else
      {
        analogWrite(PWMpin,0);
      }
      s=-1; /*we are done with state 61.  go back to state -1 */
      break;

      /**********OK THAT'S ALL THE STUFF I ADDED 1/3/2012 R White*******/

      
      /*******CASE 70 is reading HX711*****/
      case 70:
        agv=loadcell.get_units();
        Serial.println(agv);
      s=-1;  /* we are done with loadcell so next state is -1      */
      break; /* s=70 taken care of                           */
      
      
      /* s=90 means Query Script Type (1 basic, 2 motor)     */
      case 90:
      if (val==57) { 
        /* if string sent is 99  send script type via serial */
        Serial.println(1);
      }
      s=-1;  /* we are done with this so next state is -1    */
      break; /* s=90 taken care of                           */

      /* s=340 or 341 means ANALOG REFERENCE *************** */
      
      case 340:
      /* the second received value indicates the reference,
         which is encoded as is 0,1,2 for DEFAULT, INTERNAL  
         and EXTERNAL, respectively                          */

      //CASE NO LONGER USED
      switch (val) {
        
        case 48:
        //analogReference(DEFAULT);
        break;        
        
        case 49:
        //analogReference(INTERNAL);
        break;        
                
        case 50:
        //analogReference(EXTERNAL);
        break;        
        
        default:                 /* unrecognized, no action  */
        break;
      } 

      s=-1;  /* we are done with this so next state is -1    */
      break; /* s=341 taken care of                          */


      /* ******* UNRECOGNIZED STATE, go back to s=-1 ******* */
      
      default:
      /* we should never get here but if we do it means we 
         are in an unexpected state so whatever is the second 
         received value we get out of here and back to s=-1  */
      
      s=-1;  /* go back to the initial state, break unneeded */



    } /* end switch on state s                               */

  } /* end if serial available                               */
  
} /* end loop statement                                      */

void doEncoder1()
{

  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
   
  rawvel=1000000/(micros()-lastencoderedge);
  lastencoderedge=micros();  
  
  if (digitalRead(APin) != digitalRead(BPin)) {
    rawvel=rawvel*(-1);
    position1--;
  }
  else {
    position1++;
  }
  
}
