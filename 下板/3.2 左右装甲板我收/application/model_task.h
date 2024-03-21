#ifndef _MODEL_TASK_H
#define _MODEL_TASK_H

#include<stdlib.h>
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rm_pid.h"
#include "can1_receive.h"
#include "chassis_task.h"
#include "vision.h"
#include "rc.h"
#include "model_task.h"
#include "BMI088_INT.h"
#include "BMI088Middleware.h"
#include "ins_task.h"
#include "kalman.h"
#include "kalman_filter.h"

typedef struct{
	int time_flag ;
	int bee_flag ;
	int vis_time;
	int flag_yaw;
	int flag_pit;
	float speed_y;
	float speed_x;
	float setx;
	float sety;
  float wait_y;
	float wait_x;
	
}model;

typedef struct  //视觉目标速度测量
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
}speed_calc_data_t;


void model_task (void *pvParameters) ;
void bmi_bee(void);
void vis_get(void);
void vis_cruise(void);
void GIMBAL_AUTO_Mpde_Ctrl(void);
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
model*gettasksend(void);
	
#endif

