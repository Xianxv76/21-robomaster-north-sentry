#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "JudgeSystem.h"


int  ccc = 0;


int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(168);		//初始化延时函数
	usart6_init();
	
	while(1)
	{
		ccc++;
		delay_ms(10);
		
	}
	
}





