#include "incl/pid.h"

float pid (float target, float actual_value, float stepTime, struct PID_parameters* param)
{
  float error = target - actual_value;
  
  float P = error * param->kp;
  param->I += (error * stepTime) * param->ki;
  float D = ((error - param->previousError) / stepTime) * param->kd;

  float corr = P + param->I + D;
  
  param->previousError = error;
  
  if (corr > param->maxCorr)
    corr = param->maxCorr;
  else if (corr < ((-1)*(param->maxCorr)) )
    corr = (-1)*(param->maxCorr);
  return corr;
}

void init_pid_param (struct PID_parameters* param, float kp, float ki, float kd, float I, float maxCorr, float previousError)
{
  param->kp=kp;
  param->ki=ki;
  param->kd=kd;
  param->I=I;
  param->maxCorr=maxCorr;
  param->previousError=previousError;
  
  return;
}

void init_pid_kvalues (struct PID_parameters* param, float kp, float ki, float kd)
{
  param->kp=kp;
  param->ki=ki;
  param->kd=kd;
    
  return;
}
