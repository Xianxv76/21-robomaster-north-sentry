#ifndef _BUTTON_TASK_H_
#define _BUTTON_TASK_H_

#include "sys.h"
#include "button.h"
#include "FreeRTOS.h"
#include "task.h"


typedef struct
{
  u16 get;  		//实现归零的标志位
	u16 io;  		  //io口电平
	u16 time; 		//高电平次数
	u16 flag;	 	  //触发标志位
	
}button;  //按键信息

void button_get_task(void *pvParameters);

button*getbutton(void);

#endif

