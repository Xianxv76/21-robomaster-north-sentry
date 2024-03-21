#ifndef _BUTTON_TASK_H_
#define _BUTTON_TASK_H_

#include "sys.h"
#include "button.h"
#include "FreeRTOS.h"
#include "task.h"


typedef struct
{
  u16 get;  		//ʵ�ֹ���ı�־λ
	u16 io;  		  //io�ڵ�ƽ
	u16 time; 		//�ߵ�ƽ����
	u16 flag;	 	  //������־λ
	
}button;  //������Ϣ

void button_get_task(void *pvParameters);

button*getbutton(void);

#endif

