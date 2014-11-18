//               0     1     2     3    4    5    6
//fuzzy member=[ nb ,  nm  , ns  , zo , ps , pm , pb]
//      ----------\    /\    /\   /\   /\   /\   /------------
//                 \  /  \  /  \ /  \ /  \ /  \ /  
//                  X     X     X    X    X    X     
//                 / \   / \   / \  / \  / \  / \     
//                /   \ /   \ /   \/   \/   \/   \     
//-------------------------------------------------------------------> x
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
int count =0;

MPU6050 MPU6050;
int16_t ax,ay,az;
int16_t gx,gy,gz;
float aax,aay,aaz;
float ggx,ggy,ggz;
double Angle_accX;
double Gx = 0;

//===========Kalman Filter Parameter===========
const double Q_angle = 0.001;
const double Q_gyroBias = 0.003;
const double R_angle = 0.03;
unsigned long lastTime;
double dt;
double P00=10,P01=0,P10=0,P11=10,K[2];
double newAngle,newRate,angle,rate,bias = 0;
double y,S;
//============================================

//============Fuzzy Controller Parameter======
double fuzzy[7]={ -1 ,-0.35 ,-0.15,0 ,0.15 ,0.35 , 1};
int tempmember[2]={8,8};

int xmember[2]={8,8};
int ymember[2]={8,8};

double xhigh[2]={0.0,0.0};
double yhigh[2]={0.0,0.0};

int ruleresult[4]={8,8,8,8};

double minhigh[4]={0,0,0,0};
double wxu;
double w;
double u;
//==========================================
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
  angle = atan(aax/sqrt(aaz*aaz+aay*aay))*180/3.14;  Angle_accX = angle;
  bias = gx/131.0;
  lastTime = micros();
}

void loop(){
  
  Kalmanfilter();
  angle /= 40 ;
//  rate /= 
//  findruleresult(rate,angle);
//  minhighop();
//  avargeu();
  
  Serial.print(rate);
  Serial.print(",");
  Serial.println(angle);
  delay(100);

}

void Kalmanfilter(){
  MPU6050.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  //==============================
  aax = ax/16384.00;
  aay = ay/16384.00;
  aaz = az/16384.00;
  //==============================
  newAngle = atan(aax/sqrt(aaz*aaz+aay*aay))*180/3.14;  Angle_accX = newAngle;
  newRate = gx / 131.00;
  dt = ((double)(micros()-lastTime)/1000000);
  Gx=Gx+(newRate-bias)*dt;

  rate = newRate - bias;
  angle += dt * rate;

  
  P00 += dt * (dt*P11 -  P01 - P10 + Q_angle);
  P01 -= dt * P11;
  P10 -= dt * P11;
  P11 += Q_gyroBias * dt;

  
  y=newAngle -angle;
  S = P00 + R_angle;

  
  K[0] = P00 / S;
  K[1] = P10 / S;
  
  angle += K[0] * y;
  bias += K[1] * y;

  
  P00 -= K[0] *P00;
  P01 -= K[0] *P01;
  P10 -= K[1] *P00;
  P11 -= K[1] *P01;

  lastTime = micros();
}
void avargeu(){
  wxu=0;
  w=0;
  for(int i=0;i<4;i++){
    wxu+=fuzzy[ruleresult[i]]*minhigh[i];
    w+=minhigh[i];
  }
  u=wxu/w;
}
  
void minhighop(){
  int count=0;
  for(int i=0;i<2;i++){
    for(int j=0;j<2;j++){
      if(xhigh[i]==0 || yhigh[j]==0){minhigh[count]=0;count+=1;}
      else{minhigh[count]=min(xhigh[i],yhigh[j]);count+=1;}
    }
  }
//  Serial.print("minhigh:");Serial.print(minhigh[0]);  Serial.print(",");  Serial.print(minhigh[1]);Serial.print(",");
//  Serial.print(minhigh[2]);  Serial.print(",");  Serial.println(minhigh[3]);
}

void findruleresult(float x,float y){
    classify(x);
    xmember[0]=tempmember[0];
    xmember[1]=tempmember[1];
    if(xmember[0]==8 || xmember[1]==8){ xhigh[0]=1; xhigh[1]=1;}
    else{ for(int i=0;i<2;i++){ xhigh[i]=memberhigh(x,xmember[i]); } } 
    
    classify(y);
    ymember[0]=tempmember[0];
    ymember[1]=tempmember[1];
    if(ymember[0]==8 || ymember[1]==8){ yhigh[0]=1; yhigh[1]=1;}
    else{for(int i=0;i<2;i++){ yhigh[i]=memberhigh(y,ymember[i]); } } 
    
//    Serial.print("xmember:");Serial.print(xmember[0]);  Serial.print(",");  Serial.println(xmember[1]);
//    Serial.print("xmember high:");Serial.print(xhigh[0]);  Serial.print(",");  Serial.println(xhigh[1]);    
//    Serial.print("ymember:");Serial.print(ymember[0]);  Serial.print(",");  Serial.println(ymember[1]);
//    Serial.print("ymember high:");Serial.print(yhigh[0]);  Serial.print(",");  Serial.println(yhigh[1]);
    
    fuzzyrule(xmember,ymember);
//    Serial.print(ruleresult[0]);  Serial.print(",");  Serial.print(ruleresult[1]);Serial.print(",");
//    Serial.print(ruleresult[2]);  Serial.print(",");  Serial.println(ruleresult[3]);
}//void findruleresult(float x,float y)


void classify(float nowx){
  if(fuzzy[0] >= nowx ){tempmember[0]=0;tempmember[1]=8;}
  if(fuzzy[1] >= nowx && nowx> fuzzy[0]){tempmember[0]=0;tempmember[1]=1;}
  if(fuzzy[2] >= nowx && nowx> fuzzy[1]){tempmember[0]=1;tempmember[1]=2;}
  if(fuzzy[3] >= nowx && nowx> fuzzy[2]){tempmember[0]=2;tempmember[1]=3;}
  if(fuzzy[4] >= nowx && nowx> fuzzy[3]){tempmember[0]=3;tempmember[1]=4;}
  if(fuzzy[5] >=nowx && nowx> fuzzy[4]){tempmember[0]=4;tempmember[1]=5;}
  if(fuzzy[6] >=nowx && nowx> fuzzy[5]){tempmember[0]=5;tempmember[1]=6;}
  if(fuzzy[6] < nowx){tempmember[0]=6;tempmember[1]=8;}
}//void classify(float nowx)

float memberhigh(float nowx,int member){
  if(nowx<fuzzy[member]){
    float high=(nowx-fuzzy[member-1])/(fuzzy[member]-fuzzy[member-1]);
    return high;
  }
  if(nowx>fuzzy[member]){
    float high=(nowx-fuzzy[member+1])/(fuzzy[member]-fuzzy[member+1]);
    return high;
  }
  if(nowx==fuzzy[member]){
    float high=1;
    return high;
  }
}//float memberhigh(float nowx,int member)

void fuzzyrule(int xin[],int yin[]){
  int count=0;
  for(int i=0;i<2;i++){
    for(int j=0;j<2;j++){
      int x=xin[i]; int y=yin[j]; 
      if(x==0 && y==0){ruleresult[count]=6;count+=1;}if(x==1 && y==0){ruleresult[count]=6;count+=1;}
      if(x==0 && y==1){ruleresult[count]=6;count+=1;}if(x==1 && y==1){ruleresult[count]=6;count+=1;}
      if(x==0 && y==2){ruleresult[count]=5;count+=1;}if(x==1 && y==2){ruleresult[count]=5;count+=1;}
      if(x==0 && y==3){ruleresult[count]=5;count+=1;}if(x==1 && y==3){ruleresult[count]=5;count+=1;}
      if(x==0 && y==4){ruleresult[count]=4;count+=1;}if(x==1 && y==4){ruleresult[count]=4;count+=1;}
      if(x==0 && y==5){ruleresult[count]=4;count+=1;}if(x==1 && y==5){ruleresult[count]=3;count+=1;}
      if(x==0 && y==6){ruleresult[count]=3;count+=1;}if(x==1 && y==6){ruleresult[count]=2;count+=1;}
    //------------------------------------------------------------------------------------------
      if(x==2 && y==0){ruleresult[count]=6;count+=1;}if(x==3 && y==0){ruleresult[count]=6;count+=1;}
      if(x==2 && y==1){ruleresult[count]=5;count+=1;}if(x==3 && y==1){ruleresult[count]=5;count+=1;}
      if(x==2 && y==2){ruleresult[count]=4;count+=1;}if(x==3 && y==2){ruleresult[count]=4;count+=1;}
      if(x==2 && y==3){ruleresult[count]=4;count+=1;}if(x==3 && y==3){ruleresult[count]=3;count+=1;}
      if(x==2 && y==4){ruleresult[count]=3;count+=1;}if(x==3 && y==4){ruleresult[count]=2;count+=1;}
      if(x==2 && y==5){ruleresult[count]=2;count+=1;}if(x==3 && y==5){ruleresult[count]=1;count+=1;}
      if(x==2 && y==6){ruleresult[count]=1;count+=1;}if(x==3 && y==6){ruleresult[count]=0;count+=1;}
    //------------------------------------------------------------------------------------------  
      if(x==4 && y==0){ruleresult[count]=5;count+=1;}if(x==5 && y==0){ruleresult[count]=4;count+=1;}
      if(x==4 && y==1){ruleresult[count]=4;count+=1;}if(x==5 && y==1){ruleresult[count]=3;count+=1;}
      if(x==4 && y==2){ruleresult[count]=3;count+=1;}if(x==5 && y==2){ruleresult[count]=2;count+=1;}
      if(x==4 && y==3){ruleresult[count]=2;count+=1;}if(x==5 && y==3){ruleresult[count]=1;count+=1;}
      if(x==4 && y==4){ruleresult[count]=2;count+=1;}if(x==5 && y==4){ruleresult[count]=1;count+=1;}
      if(x==4 && y==5){ruleresult[count]=1;count+=1;}if(x==5 && y==5){ruleresult[count]=0;count+=1;}
      if(x==4 && y==6){ruleresult[count]=0;count+=1;}if(x==5 && y==6){ruleresult[count]=0;count+=1;}
    //------------------------------------------------------------------------------------------
      if(x==6 && y==0){ruleresult[count]=3;count+=1;}
      if(x==6 && y==1){ruleresult[count]=2;count+=1;}
      if(x==6 && y==2){ruleresult[count]=2;count+=1;}
      if(x==6 && y==3){ruleresult[count]=1;count+=1;}
      if(x==6 && y==4){ruleresult[count]=1;count+=1;}
      if(x==6 && y==5){ruleresult[count]=0;count+=1;}
      if(x==6 && y==6){ruleresult[count]=0;count+=1;}
    }//for(int j=0;j<2;j++)
  }//for(int i=0;i<2;i++)
}//void fuzzyrule(int x,int y){
  
  
  








