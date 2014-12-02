#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Timer.h"

Timer t;
float timeChange=20;
float dt=timeChange*0.001;

//=== accelgyro Part Set =============
float angleAx,gyroGy;
MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;
float K1 =0.03; 
float angle;
//====================================


//==== Encoder Pin  ===============
const int pin_R2 = 9 , pin_L2 = 7 ;   //  B signal of two encoder
volatile unsigned int R1 , R2 ; // Right motor encoder (Mark as A)
volatile unsigned int L1 , L2 ; // Left motor encoder (Mark as B)
volatile long ApositiveCount = 0 , AnagtiveCount = 0; //Right motor encoder count
volatile long BpositiveCount = 0 , BnagtiveCount = 0; //left motor encoder count

//====================================

//=== Motor Pin =====================
int MotorA1 = 6 , MotorA2 = 5;    // Right(A) motor pin
int MotorB1 = 11 , MotorB2 = 10;  //Left (B) motor pin
//===================================

//====== Encoder PID controller ===============
float En_Kp = 0.2, En_Ki = 1.3, En_Kd = 0.02;
float En_LastErr = 0;      
float En_Err = 0;         
float En_Intergral = 0;
float En_Derivation = 0;
float Av,Bv;
//=============================================

//====== Inverted Pendulum  PID controller ====
float IP_Kp = 7 , IP_Ki = 1.2 , IP_Kd = 2;
float IP_input;
float IP_reference = -6;
float IP_LastErr = 0;      
float IP_Err = 0;          
float IP_Intergral = 0;
float IP_Derivation = 0;
float Right_u = 0;
float Left_u = 0;
//=============================================

//========== BT command =======================
int command ;
int ledPin = 13;
int Ledstate = 0;
//=============================================

void setup(){
  Wire.begin();
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  accelgyro.initialize();
  
  //===== PinMode =============================
  pinMode(pin_R2,INPUT);  
  pinMode(pin_L2,INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  
  pinMode(MotorA1,OUTPUT);  
  pinMode(MotorA2,OUTPUT);
  pinMode(MotorB1,OUTPUT);  
  pinMode(MotorB2,OUTPUT);
  //===========================================
  attachInterrupt(0 , EncoderA , CHANGE); //Right(A) encoder
  attachInterrupt(1 , EncoderB , CHANGE); //left(B) encoder
  
  
  int tickEvent1 = t.every(timeChange , getangle);
  // int tickEvent2 = t.every(10 , EncoderLoop);
  int tickEvent3 = t.every(timeChange , printout);
  int tickEvent4 = t.every(timeChange , BTcommand);
}

void loop(){
  t.update();
}

void printout()
{
    Serial.print(Right_u); Serial.print(",");
    Serial.print(Left_u);  Serial.print(",");
    Serial.print(angleAx); Serial.print(",");
    Serial.print(angle);  Serial.print(",");
    Serial.print(Av); Serial.print(",");
    Serial.println(-Bv);
}
/*void EncoderLoop(){
  float Av = 0 , Bv = 0; 
  Av = (ApositiveCount - AnagtiveCount) ; 
  Bv = (BpositiveCount - BnagtiveCount) ; 
  ApositiveCount = 0;  AnagtiveCount = 0;  
  BpositiveCount = 0;  BnagtiveCount = 0; 

  En_Err = ( Av - Bv);
  En_Intergral = (0.6 * En_Intergral + En_Err);
  En_Derivation = (En_Err - En_LastErr) / 10*0.001;

  Left_u = Right_u + En_Kp * En_Err + En_Ki * En_Intergral + En_Kd * En_Derivation;

  if(Left_u < 0 && Left_u > -45){ Left_u = -45;  }
  if(Left_u > 0 && Left_u < 45 ){ Left_u = 45;   }
  if(Left_u >  255) {Left_u = 255;  }
  if(Left_u < -255) {Left_u = -255; }

  DriveLeft();
  En_LastErr = En_Err;
}*/

void getangle() {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    angleAx=atan2(ax,-az)*180/PI;
    gyroGy=gy/131.00;
    angle = K1 * angleAx+ (1-K1) * (angle + gyroGy * dt);
    //========= IP_PID Part ====================
    IP_Err = IP_reference - angle;
    IP_Intergral = (3/5) * IP_Intergral + IP_Err ;
    IP_Derivation = (IP_Err - IP_LastErr) /dt;
    Right_u = IP_Kp * IP_Err + IP_Ki * IP_Intergral + IP_Kd * IP_Derivation ;

    if(Right_u < 0 && Right_u > -45){ Right_u = -45;  }
    if(Right_u > 0 && Right_u < 45 ){ Right_u = 45;   }
    if(Right_u >  255) {Right_u = 255;  }
    if(Right_u < -255) {Right_u = -255; }

    IP_LastErr = IP_Err;
    DriveRight();  

    Av = 0 , Bv = 0; 
    Av = (ApositiveCount - AnagtiveCount) ; 
    Bv = (BpositiveCount - BnagtiveCount) ; 
    ApositiveCount = 0;  AnagtiveCount = 0;  
    BpositiveCount = 0;  BnagtiveCount = 0; 

    En_Err = ( Av + Bv);
    En_Intergral = (0.6 * En_Intergral + En_Err);
    En_Derivation = (En_Err - En_LastErr) / dt;

    Left_u =  Right_u + En_Kp * En_Err + En_Ki * En_Intergral + En_Kd * En_Derivation;

    if(Left_u < 0 && Left_u > -45){ Left_u = -45;  }
    if(Left_u > 0 && Left_u < 45 ){ Left_u = 45;   }
    if(Left_u >  255) {Left_u = 255;  }
    if(Left_u < -255) {Left_u = -255; }
    
    DriveLeft();
    En_LastErr = En_Err;
}

void DriveRight(){
  if( Right_u > 0 ){
    analogWrite(MotorA1, 0);
    analogWrite(MotorA2, Right_u);
  }
  else{
    analogWrite(MotorA1, (-Right_u));
    analogWrite(MotorA2, 0);
  }
}
 
void DriveLeft(){
  if( Left_u > 0 ){
    analogWrite(MotorB1, 0);
    analogWrite(MotorB2, Left_u);
  }
  else{
    analogWrite(MotorB1, (-Left_u));
    analogWrite(MotorB2, 0);
  }
}
 

void EncoderA(){
  R1 = digitalRead(2);  R2 = digitalRead(pin_R2); 
  if( R1 && R2){    ApositiveCount ++;  }
  else if( !R1 && !R2){    ApositiveCount ++;  }
  else if( !R1 && R2){    AnagtiveCount ++;  }
  else if ( R1 && !R2){    AnagtiveCount ++;  }
}

void EncoderB(){
  L1 = digitalRead(3);  L2 = digitalRead(pin_L2); 
  if( L1 && L2){    BpositiveCount ++;  }
  else if( !L1 && !L2){    BpositiveCount ++;  }
  else if( !L1 && L2){    BnagtiveCount ++;  }
  else if ( L1 && !L2){    BnagtiveCount ++;  }
}

void BTcommand(){
  if(Serial.available() > 0){
    if(Ledstate == 0){
      digitalWrite(ledPin,HIGH);
      Ledstate = 1;
    }
    else{
      digitalWrite(ledPin,LOW);
      Ledstate = 0;
    }
    command = Serial.read();

    if(command == 255){
      IP_LastErr = 0;      
      IP_Err = 0;          
      IP_Intergral = 0;
      IP_Derivation = 0;
    
      IP_Kp = Serial.read() / 10.0;
      IP_Ki = Serial.read() / 10.0;
      IP_Kd = Serial.read() / 10.0;
    }
    else if(command == 'L'){
      IP_reference -= 0.01;
    }
    else if(command == 'R'){
      IP_reference += 0.01;
    }
    else if(command == 'Z'){
      IP_reference = 0;
    }
    else if(command == 'X'){
      IP_reference -= 0.3;
    }
    else if(command == 'Y'){
      IP_reference += 0.3;
    }
      
  }
}