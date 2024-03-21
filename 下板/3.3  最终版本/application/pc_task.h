#ifndef _PC_TASK_H
#define _PC_TASK_H

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can1_receive.h"
#include "chassis_task.h"
#include "led.h"


void pc_task(void *pvParameters);


#endif
