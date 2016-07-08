/*
PID .h
*/
#ifndef __PID_H
#define __PID_H
#ifdef __cplusplus
 extern "C" {
#endif

   #include "stm32f0xx_hal.h"
  //float PI = 3.1415;
  //float pitch;
  //float roll;
  float *PIDRoll,*PIDPitch,*PIDYaw;
  float accelx,accely,accelz,timeInterval=.001;
  float rollKp=.1,rollKi=.001,rollKd=.001;
  float rollPout,rollIout,rollDout;
  float rollError,rollLastError,rollNewArea,rollRunningArea;
  float pitchKp=.1,pitchKi=.001,pitchKd=.001;
  float pitchPout,pitchIout,pitchDout;
  float pitchError,pitchLastError,pitchNewArea,pitchRunningArea;
  float pitchOut,rollOut;
  float pitchTarget=0,rollTarget=0,yawTarget=0;
  float motorRP,motorRN,motorPP,motorPN;
  /*
  function protos
  */
  void stabilize();
#ifdef __cplusplus
}
#endif
#endif /*__ PID_H */