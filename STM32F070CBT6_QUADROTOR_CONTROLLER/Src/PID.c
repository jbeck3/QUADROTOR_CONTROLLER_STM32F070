/*
PID CONTROL .c
*/

#include "PID.h"
#include "math.h"
//#include "IMU.h"


void setPitchTarget(float ptar){
pitchTarget = ptar;
}

void setYawTarget(float pyaw){
yawTarget = pyaw;
}

void setRollTarget(float proll){
rollTarget = proll;
}


void stabilize(){
  getAllVals();
  PIDRoll = (float *)getRoll();
  PIDPitch = (float *)getPitch();
  PIDYaw = (float *)getYaw();
  //roll PID
  rollError = rollTarget - *PIDRoll;
  rollPout = rollError * rollKp;
  rollNewArea = (rollLastError+rollError)*(0.5)*(timeInterval)*(rollKi);
  rollRunningArea += rollNewArea;
  rollIout = rollRunningArea;  
  rollDout = (rollError-rollLastError)*(rollKd)/(timeInterval);  
  rollLastError = rollError;
  rollOut = rollPout + rollIout + rollDout;
  
  motorRP = 50 + rollOut;
  motorRN = 50 - rollOut;
  
  //pitchPID
  pitchError = pitchTarget - *PIDPitch;
  pitchPout = pitchError * pitchKp;
  pitchNewArea = (pitchLastError+pitchError)*(0.5)*(timeInterval)*(pitchKi);
  pitchRunningArea += pitchNewArea;
  pitchIout = pitchRunningArea;  
  pitchDout = (pitchError-pitchLastError)*(pitchKd)/(timeInterval);  
  pitchLastError = pitchError;
  pitchOut = pitchPout + pitchIout + pitchDout;
  
  motorPP = 50 + pitchOut;
  motorPN = 50 - pitchOut;
  
  
  //now change pwm vals
  TIM1->CCR1 = motorPP + 70;
  TIM1->CCR2 = motorPN + 70;
  TIM3->CCR1 = motorRP + 70;
  TIM3->CCR2 = motorRN + 70;
  
  
}