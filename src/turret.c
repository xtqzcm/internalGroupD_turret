#include "ch.h"
#include "hal.h"
#include "dbus.h"
#include "canBusProcess.h"
#include "pid.h"
#include "turret.h"

/*
 * there is a pointer named beaten indicates the state of the fist
 * ->...  *beaten == 1, means for the current position, it has beaten the target
 * ->...  *beaten == 0, means for the current position, it hasn't beat the target
 *
 * once we run the function "get_angle_sp()" and sensed the change of the
 *       position, we will set the *beaten to be 0,
 * then we run the function "hit()", s.t. the fist hit the target and change the
 *       state of the *beaten to be 1.
 * ........
 * ........(things like that...)
 */

void hit(int* beaten)
{
  if(*beaten == 0)
  {
    // via pin A2
    //chThdSleepMilliseconds(1200);
    palTogglePad(GPIOA,2);
    chThdSleepMilliseconds(500);
    palTogglePad(GPIOA,2);
    *beaten = 1;
  }
}


float turret_init(void)
{
  Encoder_canStruct* encoder = can_getEncoder();
  float previous_rad = encoder->radian_angle;
  palClearPad(GPIOA,2);
  can_motorSetCurrent(0x200,-1200,0,0,0);
  chThdSleepMilliseconds(1000);
  while(true)
  {
    float current_rad = encoder->radian_angle;

    can_motorSetCurrent(0x200, -300,0,0,0);
    chThdSleepMilliseconds(800);

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
  while (iii < 10)
      {
          palTogglePad(GPIOA, GPIOA_LED);
          chThdSleepMilliseconds(100);
          iii += 1;
      }
  palClearPad(GPIOA,2);

  float turret_init_angle;
  turret_init_angle = encoder->radian_angle;  //initial with the stop point
  return turret_init_angle;
};


int get_angle_sp(int target, int* beaten, int* last_angle_sp)
{
  int motor_angle_sp = 0;  //just set a random strange integer to let it go
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

  if (motor_angle_sp != *last_angle_sp)//once it sensed the change of the state
  {                                    //change the *beaten to be 0
    *beaten = 0;
    *last_angle_sp = motor_angle_sp;
  }

  return motor_angle_sp;
}




void turret_output(PID_t* pid_angle, PID_t* pid_speed,
                    int* beaten, int* last_angle_sp,
                    float init_angle, float* output)
{
  Encoder_canStruct* encoder = can_getEncoder();
  RC_Ctl_t* rc;
  rc = RC_get();

  int angle_sp, speed_sp;
  //float output;

  angle_sp = get_angle_sp(rc->s2,beaten,last_angle_sp);  //will be changed
  pid_update(pid_angle, angle_sp + init_angle, encoder->radian_angle);

  speed_sp = (pid_angle->out / pid_angle->dt) * 60000.0 / 6.28318f;
  pid_update(pid_speed, speed_sp, encoder->speed_rpm);
  *output = pid_speed->out;

}
