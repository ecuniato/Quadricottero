struct PID_parameters
{
  float kp;
  float ki;
  float kd;

  float I;
  
  float maxCorr;
  float previousError;
};

float pid (float target, float actual_value, float stepTime, struct PID_parameters* param);
void init_pid_param (struct PID_parameters* param, float kp, float ki, float kd, float I, float maxCorr, float previousError);
void init_pid_kvalues (struct PID_parameters* param, float kp, float ki, float kd);
