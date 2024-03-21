#ifndef _ENCODER_TASK_H
#define _ENCODER_TASK_H

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"



typedef struct
{
  int diff;  		//�ϴκ���εĲ�ֵ
	int count;  		  //ת����Ȧ��
	int now_echo; 		//��α�������ֵ
	int last_echo;	 	  //�ϴα�������ֵ
	int all_echo;
	int max_echo;      //���λ��
	
}encoder0;  

void encoder0_task(void *pvParameters);
encoder0*get_encoder0_data(void);




#endif
