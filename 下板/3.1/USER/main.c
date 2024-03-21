#include "main.h"


int a= 0;

int main(void)
{ 
	int i =0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //设置系统中断优先级分组4
	delay_init(168);		     //初始化延时函数
	CAN_Configure();         //can1通讯
//  KEY_Init();            //按键
//	TIM4_PWM_Init();         //蜂鸣器
//	LED_Init();              //led灯
//	vision_init();         //视觉接收串口
//	TIM1_PWM_Init(2000,168); //摩擦轮电机初始化pwm
	rc_init();               //遥控器部分
	while(1)
	{}
		
//	bim088_spi_init();       //bmi088的spi的初始化
//	
//	while(BMI088_init())     //陀螺仪初始化 
//    { ; }
//	 a = 1 ;                 //陀螺仪初始化完成
	
//	delay_ms(500);

//		for(i = 0; i<8;i++)    //拨弹轮初始化 
//	{
//		TIM_SetCompare1(TIM1,1100);
//		TIM_SetCompare2(TIM1,1100);
//		delay_ms(500);
//	}
	
//	start();

	}
	

