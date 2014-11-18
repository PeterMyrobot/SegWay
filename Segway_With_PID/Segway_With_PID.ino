#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Timer.h"

Timer t;
float timeChange=20;
float dt=timeChange*0.001;
float encodertime = 10;

float angleAx,gyroGy;
MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;

float K1 =0.03; 
float angle1;


//====== PID controller====
float Kp = 0;
float Ki = 0;
float Kd = 0;
float input;
float reference = 0;
float ThetaLastErr = 0;      //float OmegaLastErr = 0 ;
float ThetaErr = 0;          //float OmegaErr = 0 ;
float Intergral = 0;
float Derivation = 0;
float u = 0;
//==========================
//======== Motor Pin========
int MotorA = 6; //When angle is negative
int MotorB = 5; //When angle is positive
int EnablePin = 9;
//==========================
//========== BT command ====
int command ;
int ledPin = 4;
int Ledstate = 0;
//==========================
//========== Encoder =======
//unsigned long currentTime;
//unsigned long loopTime;
//const int pin_A = 2;  
//const int pin_B = 3;  
//unsigned char encoder_A;
//unsigned char encoder_B;
//unsigned char encoder_A_prev=0;
//int fadeAmount = 0;
//int fadeAmount_prev = 0;
//int skipCount = 0;
//double Velocity;
//===========================



void setup() {
    Wire.begin();//初始化
    Serial.begin(9600);//初始化
    
    pinMode(ledPin,OUTPUT);
    digitalWrite(ledPin,LOW);
    pinMode(EnablePin,OUTPUT);
    digitalWrite(EnablePin,HIGH);
    
//    pinMode(pin_A, INPUT);
//    pinMode(pin_B, INPUT);
//    currentTime = millis();
//    loopTime = currentTime; 
    
    accelgyro.initialize();//初始化

    int tickEvent1 = t.every(timeChange, getangle);

    int tickEvent2 = t.every(timeChange, printout) ;
    
    int ticlEvent3 = t.every(timeChange, BTcommand) ;
    
//    int tickEvent4 = t.every(encodertime,CalculatorV);
}
void loop() {

    t.update();
//    currentTime = millis();
//    if(currentTime >= (loopTime + 5)){        // 5ms since last check of encoder = 200Hz  
//      encoder_A = digitalRead(pin_A);    
//      encoder_B = digitalRead(pin_B);   
//      if((!encoder_A) && (encoder_A_prev)){   // A has gone from high to low 
//        if(encoder_B) {                       // B is high so clockwise
//          fadeAmount ++;   
//        }   
//        else {                                // B is low so counter-clockwise      
//          fadeAmount --;               
//        }
//      }   
//      encoder_A_prev = encoder_A;     // Store value of A for next time    
//      loopTime = currentTime;  // Updates loopTime
//    }
  }
  
//void CalculatorV(){
//  if(fadeAmount != fadeAmount_prev){
//    Serial.print(fadeAmount);
//    Velocity = (fadeAmount - fadeAmount_prev)/( skipCount* 0.001 * encodertime);
//    fadeAmount_prev = fadeAmount;
//    skipCount = 1;
//  }else{
//    skipCount ++;
//    Velocity = 0;
//  }
//}

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
      float ThetaLastErr = 0;      
      float ThetaErr = 0;          
      float Intergral = 0;
      float Derivation = 0;
      float u = 0;
      Kp = Serial.read() / 10.0;
      Ki = Serial.read() / 10.0;
      Kd = Serial.read() / 10.0;
    }
    else if(command == 'L'){
      reference -= 0.01;
    }
    else if(command == 'R'){
      reference += 0.01;
    }
    else if(command == 'Z'){
      reference = 0;
    }
    else if(command == 'X'){
      reference -= 0.3;
    }
    else if(command == 'Y'){
      reference += 0.3;
    }
      
  }
}
void printout()
{
    Serial.print(gyroGy); Serial.print(",");
    Serial.print(angle1);  Serial.print(",");
    Serial.println(u);

}

  
void getangle() 
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    angleAx=atan2(ax,az)*180/PI;
    gyroGy=-gy/131.00;
    angle1 = K1 * angleAx+ (1-K1) * (angle1 + gyroGy * dt);
    //==========PID==========================
    ThetaErr = reference - angle1;
//    OmegaErr = 0 - gyroGy ;
    Intergral = (3/5) * Intergral + ThetaErr ;
    Derivation = (ThetaErr - ThetaLastErr) /dt;
    u = Kp * ThetaErr +Ki * Intergral + Kd * Derivation ;
//    ThetaLastErr = ThetaErr;

    /*=============增量PID
    Theta_N_Err = reference - angle1;
    u += Kp * (Theta_N_Err - Theta_N1_Err) +Ki * Theta_N_Err + Kd * (Theta_N_Err - 2*Theta_N1_Err + Theta_N2_Err);
    Theta_N2_Err = Theta_N1_Err;
    Theta_N1_Err = Theta_N_Err;
    */
    
    Drive();
}

void Drive(){
  if(u > 150){
    analogWrite(MotorA,0);
    analogWrite(MotorB,255);
  }
  else if(u<-150){
    analogWrite(MotorA,255);
    analogWrite(MotorB,0);
  }
  else if(u < -60 && u>-150){
    analogWrite(MotorA,-u);
    analogWrite(MotorB,0);
  }
  else if(u > 60 && u < 150){
    analogWrite(MotorA,0);
    analogWrite(MotorB,u);
  }
  else if(u > 0 && u < 60){
    analogWrite(MotorA,0);
    analogWrite(MotorB,60);
  }
  else if(u < 0 && u > -60){
    analogWrite(MotorA,60);
    analogWrite(MotorB,0);
  }  

}





