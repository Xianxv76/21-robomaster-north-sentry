#ifndef _CAN1_SEND_H
#define _CAN1_SEND_H

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_can1.h"
#include "can1_receive.h"
#include "chassis_task.h"
#include "rc.h"





void can1_send(void *pvParameters);

#endif



