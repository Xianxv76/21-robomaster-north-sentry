#include "encoder_task.h"



 encoder0 encoder_cm = {0,0,0,0,0,0};

 /*
�������ݣ���������λ����
��ڲ��� ��
����ֵ�� ��
*/


void encoder0_task(void *pvParameters)
{
	while(1)
		
	{ 	
		encoder_cm.diff = encoder_cm.now_echo - encoder_cm.last_echo;
		
		encoder_cm.last_echo = encoder_cm.now_echo;
		
		if(encoder_cm.diff < -3000)    //���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�
	{
		encoder_cm.count++;
		encoder_cm.all_echo += ( encoder_cm.diff + 6000 );
	}
	else if(encoder_cm.diff > 3000)
	{
		encoder_cm.count--;
		encoder_cm.all_echo += ( encoder_cm.diff - 6000 );
	}		
	else
	{
		encoder_cm.all_echo += encoder_cm.diff;
	}

		 vTaskDelay(1);	
	}}

	
 /*
�������ݣ���������Ϣ���غ���
��ڲ��� ��
����ֵ�� �ṹ��encoder0ָ��
*/


encoder0*get_encoder0_data(void)
{
	return &encoder_cm;	
}




