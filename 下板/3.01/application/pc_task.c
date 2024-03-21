#include "pc_task.h"

u8 USART_TX_BUF[10] ;

judgement*jud;

static flag*chassic;


void pc_task(void *pvParameters)
{
	vTaskDelay(1000);
	vTaskDelay(1000);
	vTaskDelay(1000);
	vTaskDelay(1000);
	vTaskDelay(1000);
	vTaskDelay(1000);
	jud = getjudugementdate();
	USART_TX_BUF[0] = 0xa5;
	USART_TX_BUF[9] = 0xee;
	if ( jud->robot_id == 107 )
	  USART_TX_BUF[2] = 'b';
	else if( jud->robot_id == 7 )
	  USART_TX_BUF[2] = 'r';
	else
	{
		while(1)
		{
      BLUE = 1;
    	GREEN = 1;
	    RED = 1;
			vTaskDelay(2000);
		}
	}
		
	while(1)
	{
		  int t;
			for(t=0;t < 10; t++)
			{
				USART_SendData(USART6, USART_TX_BUF[t]); 
        while(USART_GetFlagStatus(USART6,USART_FLAG_TC)!=SET);				
			}		
		vTaskDelay(1000);
	}
}


