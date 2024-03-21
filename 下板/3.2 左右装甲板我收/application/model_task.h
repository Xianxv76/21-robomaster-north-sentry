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

typedef struct  //�Ӿ�Ŀ���ٶȲ���
{
  int delay_cnt;//����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
  int freq;
  int last_time;//�ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
  float last_position;//�ϸ�Ŀ��Ƕ�
  float speed;//�ٶ�
  float last_speed;//�ϴ��ٶ�
  float processed_speed;//�ٶȼ�����
}speed_calc_data_t;


void model_task (void *pvParameters) ;
void bmi_bee(void);
void vis_get(void);
void vis_cruise(void);
void GIMBAL_AUTO_Mpde_Ctrl(void);
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
model*gettasksend(void);
	
#endif

