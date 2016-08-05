/*
 **
 **FILE: PID CONTROL INTERFACE.c
 **AUTHOR: JBECKIII
 **DATE:6/17/2016
 **PURPOSE: PROVIDE INTERFACE FOR A PID IMPLEMENTATION
 **
 **
 */
#include "PID.h"
#include "math.h"
//#include "IMU.h"


void setPitchTarget(float ptar){
	pitchTarget = ptar;
}

void setYawTarget(float ytar){
	yawTarget = ytar;
}

void setRollTarget(float rtar){
	rollTarget = rtar;
}

void setP(float pgain){
  rollKp = pgain;
  pitchKp = pgain;
}
        
void setI(float igain){
  rollKi = igain;
  pitchKi = igain;
}

void setD(float dgain){
  rollKd = dgain;
  pitchKd = dgain;
}  

void setCent(float coff){
  ROLL_MOTOR_CENTER_VAL = coff;
  PITCH_MOTOR_CENTER_VAL = coff;
}

void stabilize(){
  if(PIDFlag){
    if(ZieglerFlag){
            rollKp=0.6*KC;
            rollKi=0.5*PC;
            rollKd=PC/8;  
    }else if(KeepStaticVals){
            rollKp=.2;
            rollKi=.025;
            rollKd=.10;
    }
    
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
    
    motorRP = ROLL_MOTOR_CENTER_VAL + OVERALLOFF + rollOut;
    motorRN = ROLL_MOTOR_CENTER_VAL + OVERALLOFF - rollOut;
    
    //pitchPID
    pitchError = pitchTarget - *PIDPitch;
    pitchPout = pitchError * pitchKp;
    pitchNewArea = (pitchLastError+pitchError)*(0.5)*(timeInterval)*(pitchKi);
    pitchRunningArea += pitchNewArea;
    pitchIout = pitchRunningArea;  
    pitchDout = (pitchError-pitchLastError)*(pitchKd)/(timeInterval);  
    pitchLastError = pitchError;
    pitchOut = pitchPout + pitchIout + pitchDout;
    
    motorPP = PITCH_MOTOR_CENTER_VAL + OVERALLOFF + pitchOut;
    motorPN = PITCH_MOTOR_CENTER_VAL + OVERALLOFF - pitchOut;
    
  }  
  
  
     
    
  //now change pwm vals
  TIM1->CCR1 = motorPP + MOTORPP_OFFSET;
  TIM1->CCR2 = motorPN + MOTORPN_OFFSET;
  TIM3->CCR1 = motorRP + MOTORRP_OFFSET;
  TIM3->CCR2 = motorRN + MOTORRN_OFFSET;
  
  
}