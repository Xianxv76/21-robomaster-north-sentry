#include "button_task.h"

static button button_key  = {0,0,0,0};
              
//static encoder0*encoder_cm;
											
/*
�������ݣ�������������
��ڲ��� ��
����ֵ�� ��
*/

void button_get_task(void *pvParameters)
{

	while(1)
	{
		button_key.io =  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
	
			if( button_key.get == 0)
			{
				 if ( button_key.io == 0)
						button_key.time ++;
				 if( button_key.time >= 3 )
				 {
					 button_key.get = 1;
					 button_key.time = 0;
					 button_key.flag ++;      //����
				 }	 
		 }	
			//��������
		 if( button_key.get == 1 )	
		 {
				if( button_key.io == 1 )
					button_key.time ++;
				if( button_key.time >=3 )
				 { 
					 button_key.get = 0;
					 button_key.time = 0;
				 }
		 }
		 vTaskDelay(10);
	}
}

/*
�������ݣ�����key������Ϣ
��ڲ��� ��
����ֵ�� ָ��ṹ��
*/
button*getbutton(void)
{
	return &button_key;
}
	 
