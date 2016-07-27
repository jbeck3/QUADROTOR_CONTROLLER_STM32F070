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

	float rollKp=.3;
	float rollKi=.035;
	float rollKd=.035;
	float rollPout,rollIout,rollDout;
	float rollError,rollLastError,rollNewArea,rollRunningArea;

	float pitchKp=.3;
	float pitchKi=.035;
	float pitchKd=.035;
	float pitchPout,pitchIout,pitchDout;
	float pitchError,pitchLastError,pitchNewArea,pitchRunningArea;

	float pitchOut,rollOut;
	float pitchTarget=0,rollTarget=0,yawTarget=0;
	float motorRP,motorRN,motorPP,motorPN;
	int MOTORPP_OFFSET=0,MOTORPN_OFFSET=0,MOTORRP_OFFSET=0,MOTORRN_OFFSET=0;
	int PITCH_MOTOR_CENTER_VAL=0,ROLL_MOTOR_CENTER_VAL=0;
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