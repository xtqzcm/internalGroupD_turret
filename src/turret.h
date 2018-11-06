#ifndef _TURRET_H_
#define _TURRET_H_

/*Handle the cylinder via magnetic valve*/
void hit(int* beaten);

/*Initialize the turret by rotating to the end*/
float turret_init(void);

/*read the rc and get angle setpoint and hit*/
int get_angle_sp(int target, int* beaten, int* last_angle_sp);

/*get the final output*/
float turret_output(PID_t* pid_angle, PID_t* pid_speed,
                    int* beaten, int* last_angle_sp);



#endif
