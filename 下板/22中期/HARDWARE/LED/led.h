#ifndef __LED_H
#define __LED_H
#include "sys.h"



//LED端口定义
#define BLUE PHout(10)
#define GREEN PHout(11)	
#define RED   PHout(12)	 

void LED_Init(void);//初始化		 				    
#endif
