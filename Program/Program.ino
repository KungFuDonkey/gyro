#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define smoothingsize 5
#define boundssize 50
#define movementbounds 350

#define restingx 170
#define restingy 255
#define restingz 480

#define sidewaysx 30
#define sidewaysy 255
#define sidewaysz 180

#define sidewaysdownx 200
#define sidewaysdowny 15
#define sidewaysdownz 210

int statearray[5]; 


MPU6050 mpu;
long ax[smoothingsize], ay[smoothingsize], az[smoothingsize];
long gx[smoothingsize], gy[smoothingsize], gz[smoothingsize];

int16_t axt;
int16_t ayt;
int16_t azt;
int16_t gxt;
int16_t gyt;
int16_t gzt;

long rx;
long ry;
long rz;

long fx;
long fy;
long fz;

#define undefined 0
#define resting 1
#define sideways 2
#define sidewaysdown 3
#define sidewaysfront 4
#define rotate 5




unsigned long prevTime;

void setup()
{
  mpu.initialize();
  Serial.begin(9600);
  Wire.begin();
  
  for(int i = 0; i < smoothingsize; i++){
    ax[i] = 0;
    ay[i] = 0;
    az[i] = 0;
    gx[i] = 0;
    gy[i] = 0;
    gz[i] = 0;
  }
  
  pinMode(13,OUTPUT);
  clearStates();
}

void loop()
{
  if(statearray[0] == resting){
    digitalWrite(13,HIGH);
  }
  else{
    digitalWrite(13,LOW);
  }
  int i = 0;
  for(int i = smoothingsize - 1; i >= 1; i--){
    ax[i] = ax[i - 1];
    ay[i] = ay[i - 1];
    az[i] = az[i - 1];
    gx[i] = gx[i - 1];
    gy[i] = gy[i - 1];
    gz[i] = gz[i - 1];
  }


  mpu.getMotion6(&axt, &ayt, &azt, &gxt, &gyt, &gzt);
  ax[0] = (long)axt;
  ay[0] = (long)ayt;
  az[0] = (long)azt;
  gx[0] = gxt;
  gy[0] = gyt;
  gz[0] = gzt;
  rx = average(ax);
  ry = average(ay);
  rz = average(az);
  fx = average2(gx);
  fy = average2(gy);
  fz = average2(gz);
  
  /*
  Serial.print(rx);
  Serial.print("\t");
  Serial.print(ry);
  Serial.print("\t");
  Serial.print(rz);
  Serial.print("\t");
  Serial.print(fx);
  Serial.print("\t");
  Serial.print(fy);
  Serial.print("\t");
  Serial.println(fz);
  */
  CheckStates();
  prevTime = millis();
}

void CheckStates(){
  CheckmovementX();
  Checkresting();
  Checksideways();
  Checksidewaysdown();
  Checksidewaysfront();
  Checkrotatestate();
}

long average(long* value){
  long answer = 0;
  for(int i = 0; i < smoothingsize; i++){
    answer += value[i];
  }
  return map(answer / smoothingsize, -17000, 17000, 0, 511);
}

long average2(long* value){
  long answer = 0;
  for(int i = 0; i < smoothingsize; i++){
    answer += value[i];
  }
  return map(answer / smoothingsize, -10000, 10000, -255, 255);
}

unsigned long restingtimer = 500;
#define TIMER 500

unsigned long sidewaystimer = 500;

unsigned long sidewaysdowntimer = 500;


unsigned long rotatetimer = 500;

void Checkresting(){ 
  if(statearray[0] != resting && CheckBounds(rx,restingx) && CheckBounds(ry,restingy) && CheckBounds(rz,restingz))
  {
    if(millis() - restingtimer >= TIMER){
      clearStates();
      changeState(resting);
      restingtimer = millis();
    }
    return;
  }
  
  restingtimer = millis();
}

void Checksideways(){
  if(statearray[0] != sideways && 
     CheckBounds(rx,sidewaysx)  && 
     CheckBounds(ry,sidewaysy)  && 
     CheckBounds(rz,sidewaysz))
  {
      if(millis() - sidewaystimer >= TIMER){
        changeState(sideways);
        sidewaystimer = millis();
      }
      return;
  }
  sidewaystimer = millis();
}

void Checksidewaysdown(){
  if(statearray[0] != sidewaysdown  && 
     statearray[0] != sidewaysfront &&
     CheckBounds(rx,sidewaysdownx)  && 
     CheckBounds(ry,sidewaysdowny)  && 
     CheckBounds(rz,sidewaysdownz))
  {
      if(millis() - sidewaysdowntimer >= TIMER){
        changeState(sidewaysdown);
        sidewaysdowntimer = millis();
      }
      return;
  }
  sidewaysdowntimer = millis();
}
bool xmovement = false;
void Checksidewaysfront(){
  if(statearray[0] == sideways && xmovement)
  {
      changeState(sidewaysfront);
      return;
  }
}


void Checkrotatestate(){
  if(statearray[0] == sidewaysfront && xmovement)
  {
     if(millis() - rotatetimer >= 1500){
      changeState(rotate);
      rotatetimer = millis();
      }
      return;
  }
  rotatetimer = millis();
}

unsigned long xmovementTimer = 500;

void CheckmovementX(){
  if(fx > movementbounds || fx < -movementbounds){
    xmovement = true;
    xmovementTimer = millis();
  }
  if(millis() - xmovementTimer >= TIMER){
    xmovement = false;
  }

}


void changeState(int value){
  for(int i = 4; i >= 1; i--){
    statearray[i] = statearray[i - 1];
  }
  statearray[0] = value;
  Serial.println(value);
}

void clearStates(){
  for(int i = 0; i < 5; i++){
    statearray[i] = undefined;
  }
}

bool CheckBounds(int16_t value, int bounds){
  return value > bounds - boundssize && value < bounds + boundssize;
}



