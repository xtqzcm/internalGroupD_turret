/********************turret****************************/
/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
/**
 * @brief This file is based on the MAPLE MINI example from ChibiOS
 *
 * @file main.cpp
 * @author TUZKI_XIANG  QAQ
 * @date 2018-11-06
 */
#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "canBusProcess.h"
#include "pid.h"
#include "turret.h"

static RC_Ctl_t *rc;
PID_t pid_angle, pid_speed;

int beaten = 0;
int last_angle_sp = 10;

//1 for angle
const float kp_1 = 40;
const float ki_1 = 0.001;
const float kd_1 = 1;
const float dt_1 = 1;
const float output_limit_1 = 30000;
const float integral_limit_1 = 10000;

//2 for speed
const float kp_2 = 0.01;
const float ki_2 = 0.0012;
const float kd_2 = 0.07;
const float dt_2 = 1;
const float output_limit_2 = 30000;
const float integral_limit_2 = 20000;


static THD_WORKING_AREA(motor_ctrl_thread_wa,512);
static THD_FUNCTION(motor_ctrl_thread, p)
{
    (void) p;

    float init_angle;
    float out = 0;
    init_angle = turret_init();

    // angle_pid_control
    pid_init(&pid_angle,
             kp_1, ki_1, kd_1, dt_1,
             output_limit_1,
             integral_limit_1);

    //speed_pid_control
    pid_init(&pid_speed,
             kp_2, ki_2, kd_2, dt_2,
             output_limit_2,
             integral_limit_2);


    while(true)
    {

      out = turret_output(&pid_angle,&pid_speed,
                          &beaten, &last_angle_sp, init_angle);

      /*TODO set motor current, channel3 for instant stop*/
      if (rc->channel3 < 600)
        can_motorSetCurrent(
                  0x200,
                  0,0,0,0);

      else if (rc->channel3 > 600)
        can_motorSetCurrent(
                0x200,
                out,0,0,0);

      chThdSleepMilliseconds(dt_1);
    }
}





/*
 * Application entry point.
 */
int main(void)
{

    /*
    * System initializations.
    * - HAL initialization, this also initializes the configured device drivers
    *   and performs the board-specific initializations.
    * - Kernel initialization, the main() function becomes a thread and the
    *   RTOS is active.
    */
    halInit();
    chSysInit();
    RC_init();
    can_processInit();
    rc = RC_get();

    chThdCreateStatic(motor_ctrl_thread_wa,
                      sizeof(motor_ctrl_thread_wa),
                      NORMALPRIO, motor_ctrl_thread, NULL);

    /*
    * Normal main() thread activity
    */
    while (true)
    {
        palTogglePad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);

    }
}




