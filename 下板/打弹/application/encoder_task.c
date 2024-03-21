#include "encoder_task.h"



 encoder0 encoder_cm = {0,0,0,0,0,0};

 /*
函数内容：编码器定位任务
入口参数 无
返回值： 无
*/


void encoder0_task(void *pvParameters)
{
	while(1)
		
	{ 	
		encoder_cm.diff = encoder_cm.now_echo - encoder_cm.last_echo;
		
		encoder_cm.last_echo = encoder_cm.now_echo;
		
		if(encoder_cm.diff < -3000)    //两次编码器的反馈值差别太大，表示圈数发生了改变
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
函数内容：编码器信息返回函数
入口参数 无
返回值： 结构体encoder0指针
*/


encoder0*get_encoder0_data(void)
{
	return &encoder_cm;	
}




