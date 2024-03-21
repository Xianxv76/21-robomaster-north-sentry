#ifndef _MODEL_TASK_H
#define _MODEL_TASK_H

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
	
//	      accel_pass[3],
//	      gyro_zero[3];
//	float gyro[3];
//  float yaw, pit, rol, yaw1, yaw_last, count;
//	float low_pass_kp;
	
}model;


void model_task (void *pvParameters) ;


#endif

