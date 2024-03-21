#ifndef __FIRE_TAKS_H
#define __FIRE_TAKS_H
#include "sys.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rc.h"
#include "vision.h"
#include "rm_pid.h"
#include "chassis_task.h"

typedef struct
{
    int jam;
    int time;
    int jam_time;
	
}flag_fire;


void fire_task (void *pvParameters);


#endif

