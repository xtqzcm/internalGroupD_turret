#include "ch.h"
#include "hal.h"
#include "dbus.h"
#include "canBusProcess.h"
#include "pid.h"
#include "turret.h"

void hit(int* beaten)
{
  if(*beaten == 0)
  {
    // via pin B4
    palSetPad(GPIOB,4);
    chThdSleepMilliseconds(800);
    palClearPad(GPIOB,4);
    *beaten = 1;
  }
}


float turret_init(void)
{
  Encoder_canStruct* encoder = can_getEncoder();
  float previous_rad = encoder->radian_angle;

  can_motorSetCurrent(0x200,-550,0,0,0);
  chThdSleepMilliseconds(1500);
  while(true)
  {
    float current_rad = encoder->radian_angle;

    can_motorSetCurrent(0x200, -300,0,0,0);
    chThdSleepMilliseconds(1500);

    if(current_rad - previous_rad < 0.1
        && current_rad - previous_rad > -0.1)  // when stop,
    {
      can_motorSetCurrent(0x200,0,0,0,0);
      break;
    }
    previous_rad = current_rad;
  }

  /*twinkle the LED to indicate the end of the initialization*/
  int iii = 0;
  while (iii < 15)
      {
          palTogglePad(GPIOA, GPIOA_LED);
          chThdSleepMilliseconds(100);
          iii += 1;
      }
  float turret_init_angle = encoder->radian_angle;  //以左打到头的电机角度为初始的基准角度。。。。。初始化
  return turret_init_angle;
};


int get_angle_sp(int target, int* beaten, int* last_angle_sp)
{
  int motor_angle_sp = 10;
  switch(target)
  {
  case 1:
    motor_angle_sp = 0;
    break;
  case 2:
    motor_angle_sp = 50;
    break;
  case 3:
    motor_angle_sp = 25;
    break;
  }
  if (motor_angle_sp != *last_angle_sp)
  {
    *beaten = 0;
    *last_angle_sp = motor_angle_sp;
  }
  return motor_angle_sp;
}




float turret_output(PID_t* pid_angle, PID_t* pid_speed,
                    int* beaten, int* last_angle_sp)
{
  Encoder_canStruct* encoder = can_getEncoder();
  RC_Ctl_t* rc;

  int angle_sp, speed_sp;
  float output;


  angle_sp = get_angle_sp(rc->s2,*beaten, *last_angle_sp);  //will be changed
  pid_update(pid_angle, angle_sp, encoder->radian_angle);

  if (pid_angle->error_now < 5.0 ||
      pid_angle->error_now > -5.0)    //check and hit
  {
    hit(*beaten);
  };

  speed_sp = (pid_angle->out / pid_angle->dt) * 60000.0 / 6.28318f;
  pid_update(pid_speed, speed_sp, encoder->speed_rpm);

  output = pid_speed->out;
  return output;
}
