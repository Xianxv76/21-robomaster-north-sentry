#include "button.h"

/*
函数内容：触碰开关检测
入口参数 无
返回值： 无
*/

void KEY_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   //普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;    //上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA0

} 

