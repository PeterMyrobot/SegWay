/* 2014 .08. 22
Two motor with encoder (AB signal)
Try to drive two motor in same speed(not calculate really rpm YET)
Using PID control to processing this problem 

Interrupt to get encoder signal
Timer to calculate speed

write by Peter
*/
#include "Timer.h"
Timer t; 
float timeChange=20;  // every 20 milliseconds calculate speed 
float dt=timeChange*0.001; // milliseconds to seconds

const int pin_R2 = 9 , pin_L2 = 4 ;   //  B signal of two encoder

volatile unsigned int R1 , R2 ; // Right motor encoder (Mark as A)
volatile unsigned int L1 , L2 ; // Left motor encoder (Mark as B)
volatile long ApositiveCount = 0 , AnagtiveCount = 0; //Right motor encoder count
volatile long BpositiveCount = 0 , BnagtiveCount = 0; //left motor encoder count

int MotorA1 = 6 , MotorA2 = 5;    // Right(A) motor pin
int MotorB1 = 11 , MotorB2 = 10;  //Left (B) motor pin
int readval; // driving Motor with register

float right_val = 0 , left_val = 0; //Command for two motor 
//====== PID controller====
float Kp = 0.2, Ki = 1.3, Kd = 0.02;
float LastErr = 0;      
float Err = 0;         
float Intergral = 0;
float Derivation = 0;
//==========================

void setup(){
  Serial.begin(9600);
  
  pinMode(R2,INPUT);  
  pinMode(L2,INPUT);
  
  pinMode(MotorA1,OUTPUT);  
  pinMode(MotorA2,OUTPUT);
  pinMode(MotorB1,OUTPUT);  
  pinMode(MotorB2,OUTPUT);
  
  attachInterrupt(0 , EncoderA , CHANGE); //Right(A) encoder
  attachInterrupt(1 , EncoderB , CHANGE); //left(B) encoder
  
  int Event1 = t.every(timeChange , CalculatorV); // Timer Event

}

void CalculatorV(){
  float Av = 0 , Bv = 0; 
  Av = (ApositiveCount - AnagtiveCount) ; //
  Bv = (BpositiveCount - BnagtiveCount) ; // Calculate Speed
  Serial.print(Av);  Serial.print(",");  Serial.print(Bv);  Serial.print(",");
  ApositiveCount = 0;  AnagtiveCount = 0;  //
  BpositiveCount = 0;  BnagtiveCount = 0;  //reset count
  
  readval = analogRead(A0);            //Driving right(A) motor with variable resistor
  readval = map(readval,0,1024,0,255); //
  right_val = readval;                 //

  Err = (Av- Bv);
  Intergral = (0.6 * Intergral + Err );
  Derivation = (Err - LastErr) /dt;
  Serial.print(" Err :");  Serial.print(Err); 
  Serial.print(" Int :");  Serial.print(Intergral);
  Serial.print(" Der :");  Serial.print(Derivation);Serial.print(",");
  left_val =  right_val + Kp * Err+Ki * Intergral + Kd * Derivation ;
  LastErr = Err;
  
  if(left_val > 255) left_val = 255;
  if(left_val < 0) left_val = 0;
  Serial.print(right_val); Serial.print(","); Serial.println(left_val);
  
  analogWrite(MotorA1,right_val);
  analogWrite(MotorA2,0);
  analogWrite(MotorB1,left_val);
  analogWrite(MotorB2,0);

}
  
// interrupt part
void EncoderA(){
  R1 = digitalRead(2);  
  R2 = digitalRead(pin_R2); 
  if( R1 && R2){
    ApositiveCount ++;
  }
  else if( !R1 && !R2){
    ApositiveCount ++;
  }
  else if( !R1 && R2){
    AnagtiveCount ++;
  }
  else if ( R1 && !R2){
    AnagtiveCount ++;
  }
}
void EncoderB(){
  L1 = digitalRead(3);
  L2 = digitalRead(pin_L2); 
  if( L1 && L2){
    BpositiveCount ++;
  }
  else if( !L1 && !L2){
    BpositiveCount ++;
  }
  else if( !L1 && L2){
    BnagtiveCount ++;
  }
  else if ( L1 && !L2){
    BnagtiveCount ++;
  }
}
  
void loop(){
  t.update(); 
}
  
