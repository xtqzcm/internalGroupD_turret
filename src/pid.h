#ifndef SRC_PID_H_
#define SRC_PID_H_

typedef struct
{
  float kp;
  float ki;
  float kd;
  float dt;

  float pout;
  float iout;
  float dout;

  float error_now;
  float error_last;

  float out;

  float output_limit;
  float integral_limit;
}PID_t;

void pid_init(PID_t* pid_struct,
              float kp, float ki, float kd, float dt,
              float output_limit, float integral_limit);

void pid_update(PID_t* pid_struct, float set, float get);

float limit(float max_value, float value);

#endif /* SRC_PID_H_ */
