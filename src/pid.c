#include "pid.h"

void pid_init(PID_t* pid_struct,
              float kp, float ki, float kd, float dt,
              float output_limit, float integral_limit)
{
  pid_struct->kp = kp;
  pid_struct->ki = ki;
  pid_struct->kd = kd;
  pid_struct->dt = dt;
  pid_struct->out = 0.0f;
  pid_struct->pout = 0.0f;
  pid_struct->iout = 0.0f;
  pid_struct->dout = 0.0f;
  pid_struct->error_now = 0.0f;
  pid_struct->error_last = 0.0f;
  pid_struct->output_limit = output_limit;
  pid_struct->integral_limit = integral_limit;
}

float limit(float max_value, float value)
{
    if (value > max_value)
      return max_value;
    else if (value < -max_value)
      return -max_value;
    else
      return value;
}

void pid_update(PID_t* pid_struct, float set, float get)
{
  pid_struct->error_now = set - get;

  pid_struct->pout  = pid_struct->kp * pid_struct->error_now;
  pid_struct->iout += pid_struct->ki * pid_struct->error_now;
  pid_struct->iout *= 0.73f;
  pid_struct->dout  = pid_struct->kd * (pid_struct->error_now
        - pid_struct->error_last) / pid_struct->dt;

  pid_struct->iout = limit(pid_struct->integral_limit, pid_struct->iout);
  pid_struct->out = pid_struct->pout + pid_struct->iout + pid_struct->dout;
  pid_struct->error_last = pid_struct->error_now;
  pid_struct->out = limit(pid_struct->output_limit, pid_struct->out);

}
