#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include<stdlib.h>
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rm_pid.h"
#include "can1_receive.h"
#include "rc.h"
#include "button_task.h"
#include "led.h"
#include "encoder_task.h"
#include "model_task.h"
#include "fire_task.h"
#include "vision.h"

#define rand_speed_max_mid 2000
#define rand_speed_mini_mid 1000

#define rand_speed_max_get 1000
#define rand_speed_mini_get 500

#define rand_time_mini 700    //ms
#define rand_time_max  1000

typedef struct
{
    int start;
    int rand_time_flag;
    int rand_time;
	  int rand_speed;
	  int rand_last_speed; 
	  int rand_speed_diff;
	  int bee_flag;
}flag;



typedef struct
{
    float standard;
    float yaw;
    float pitch;
	  float fire;
	  float yaw_cirle;
    float pitch_cirle;

}moto_send;


void chassis_task(void *pvParameters);    //任务函数
void free_move(void);                     //自由运动函数
moto_send*getmotosend(void);
void int_location(void);
void standart_mode_rand(void);
void speed_fast(void);
void speed_low(void);
flag*getchassic_flag(void);



#endif
