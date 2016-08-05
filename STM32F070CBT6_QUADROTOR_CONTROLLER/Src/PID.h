/*
 **
 **FILE: PID CONTROL INTERFACE.h
 **AUTHOR: JBECKIII
 **DATE:6/17/2016
 **PURPOSE: PROVIDE INTERFACE FOR A PID IMPLEMENTATION
 **
 **
 */
#ifndef __PID_H
#define __PID_H
#ifdef __cplusplus
extern "C" {
	#endif

	#include "stm32f0xx_hal.h"

	/*Variables and Constants*/
	float *PIDRoll,*PIDPitch,*PIDYaw;
	float accelx,accely,accelz,timeInterval=.001;
        int PIDFlag = 1;
	float rollKp=.1;
	float rollKi=.02;
	float rollKd=.0;
	float rollPout,rollIout,rollDout;
	float rollError,rollLastError,rollNewArea,rollRunningArea;

	float pitchKp=.02;
	float pitchKi=.01;
	float pitchKd=.00;
	float pitchPout,pitchIout,pitchDout;
	float pitchError,pitchLastError,pitchNewArea,pitchRunningArea;

        float Kp = .3;
        float Ki = .035;
        float Kd = .035;
        
	float pitchOut,rollOut;
	float pitchTarget=0,rollTarget=0,yawTarget=0;
	float motorRP,motorRN,motorPP,motorPN;
	int MOTORPP_OFFSET=8,MOTORPN_OFFSET=0,MOTORRP_OFFSET=2,MOTORRN_OFFSET=2;
	int PITCH_MOTOR_CENTER_VAL=100,ROLL_MOTOR_CENTER_VAL=100,OVERALLOFF=0;
	float KC=0,PC=0;
	int ZieglerFlag=0,KeepStaticVals=0;

	/*Function prototypes*/
	void stabilize();
	void setPitchTarget(float ptar);
	void setYawTarget(float ytar);
	void setRollTarget(float rtar);

	#ifdef __cplusplus
}
#endif
#endif /*__ PID_H */