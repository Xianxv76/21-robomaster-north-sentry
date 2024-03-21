#ifndef __START_TASK_H_
#define __START_TASK_H_
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "can1_send.h"
#include "chassis_task.h"
#include "button_task.h"
#include "encoder_task.h"
#include "model_task.h"
#include "BMI088_INT.h"
#include "BMI088Middleware.h"
#include "ins_task.h"


void start_task(void *pvParameters);
void start(void);

	 				    
#endif
