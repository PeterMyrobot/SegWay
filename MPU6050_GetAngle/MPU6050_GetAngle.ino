#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>

MPU6050 MPU6050;

int16_t ax,ay,az;
int16_t gx,gy,gz;
float Angel_accX,Angel_accY,Angel_accZ;
float Angel_gx,Angel_gy,Angel_gz;
float aax,aay,aaz;
float ggx,ggy,ggz;
float compx,compy;


unsigned long timer ;

void setup(){
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initializing I2C device....");
  MPU6050.initialize();
  
  Serial.println("Testing device connections...");
  Serial.println(MPU6050.testConnection()?"MPU6050 connection successful":"MPU6050 connection failure");
  
  MPU6050.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  aax = ax/16384.00;
  aay = ay/16384.00;
  aaz = az/16384.00;
  Angel_accX = atan(aax/sqrt(aaz*aaz+aay*aay))*180/3.14;
  Angel_accY = atan(aay/sqrt(aax*aax+aaz*aaz))*180/3.14;
  Angel_gx = Angel_accX;
  Angel_gy = Angel_accY;
  compx = Angel_accX;
  compy = Angel_accY;
  
  timer = micros();
}

void loop(){
  MPU6050.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  //==============================
  aax = ax/16384.00;
  aay = ay/16384.00;
  aaz = az/16384.00;
  //==============================
  Angel_accX = atan(aax/sqrt(aaz*aaz+aay*aay))*180/3.14;
  Angel_accY = atan(aay/sqrt(aax*aax+aaz*aaz))*180/3.14;
  Angel_accZ = atan(aaz/sqrt(aax*aax+aay*aay))*180/3.14;
  //==============================
  ggx = gx / 131.00;
  ggy = gy / 131.00;
  ggz = gz / 131.00;
  //==============================
  Angel_gx += ggx * ((float)(micros()-timer)/1000000);
  Angel_gy += ggy * ((float)(micros()-timer)/1000000);
  
  compx = (0.98*(compx + (ggx * ((float)(micros()-timer)/1000000))))+(0.02*Angel_accX);
  compy = (0.98*(compy + (ggy * ((float)(micros()-timer)/1000000))))+(0.02*Angel_accY);
  timer = micros();
  //============================================
//  Serial.print(Angel_accX);Serial.print(",");
//  Serial.print(Angel_accY);Serial.print(",");
//  Serial.print(Angel_gx);Serial.print(",");
  Serial.print(Angel_gy);Serial.print(",");
//  Serial.print(compx);Serial.print(",");
  Serial.println(compy);
}
  

