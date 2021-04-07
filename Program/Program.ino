#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define smoothingsize 5
#define boundssize 50
#define movementbounds 375

#define restingx 240
#define restingy 250
#define restingz 500

#define sidewaysx 20
#define sidewaysy 250
#define sidewaysz 250

#define sidewaysdownx 240
#define sidewaysdowny 7
#define sidewaysdownz 240

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

unsigned long lastStateTime;
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

bool CheckBounds(int16_t value = 0, int bounds = 0, int = boundssize);


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
  CheckGestures();
}

void CheckStates(){
  CheckmovementX();
  Checkresting();
  Checksideways();
  Checksidewaysdown();
  Checksidewaysfront();
  Checkrotatestate();
}

void CheckGestures(){
  if(statearray[0] == 3)
  {
    clearStates();
    waterPlant();
  }
   if(statearray[0] == 5)
  {
    clearStates();
    updateValues();
  }
  else if(statearray[0] == 2 && millis() - lastStateTime >= 2000)
  {
    clearStates();
    toggleMode();
  }
  else if(statearray[0] == 4 && (millis() - lastStateTime) >= 2000)
  {
    clearStates();
    nextMenu();
  }
  else
  {
    
  }
}

void waterPlant(){
  Serial.println("water plant");
}

void toggleMode(){
  Serial.println("toggle");
}

void nextMenu(){
  Serial.println("nextMenu");
}

void updateValues(){
  Serial.println("updateValues");
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

unsigned long sidewaystimer = 100;

unsigned long sidewaysdowntimer = 200;


unsigned long rotatetimer = 500;

void Checkresting(){ 
  if(statearray[0] != resting && CheckBounds(rx,restingx, 55) && CheckBounds(ry,restingy, 55) && CheckBounds(rz,restingz, 55))
  {
    if(millis() - restingtimer >= TIMER){
      CheckGestures();
      clearStates();
      changeState(resting);
      restingtimer = millis();
    }
    return;
  }
  
  restingtimer = millis();
}

void Checksideways(){
  if(statearray[0] == resting && 
     CheckBounds(rx,sidewaysx, 80)  && 
     CheckBounds(ry,sidewaysy, 80)  && 
     CheckBounds(rz,sidewaysz, 80))
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
  if((statearray[0] == resting || statearray[0] == sideways) &&
     statearray[0] != sidewaysfront                           &&
     statearray[0] != undefined                               &&
     CheckBounds(rx,sidewaysdownx)                            && 
     CheckBounds(ry,sidewaysdowny)                            && 
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
      if(millis() - lastStateTime >= 200)
      {
        changeState(sidewaysfront);
        return;
      }
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
  lastStateTime = millis();
  Serial.println(value);
}

void clearStates(){
  for(int i = 0; i < 5; i++){
    statearray[i] = undefined;
  }
}

bool CheckBounds(int16_t value, int bounds, int bsize){
  return value > bounds - bsize && value < bounds + bsize;
}
