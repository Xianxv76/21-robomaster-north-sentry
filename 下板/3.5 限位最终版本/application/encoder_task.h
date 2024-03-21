#ifndef _ENCODER_TASK_H
#define _ENCODER_TASK_H

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"



typedef struct
{
  int diff;  		//上次和这次的差值
	int count;  		  //转过的圈数
	int now_echo; 		//这次编码器的值
	int last_echo;	 	  //上次编码器的值
	int all_echo;
	int max_echo;      //最大位置
	
}encoder0;  

void encoder0_task(void *pvParameters);
encoder0*get_encoder0_data(void);




#endif
