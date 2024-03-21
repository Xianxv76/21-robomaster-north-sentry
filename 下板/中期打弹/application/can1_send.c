#include "can1_send.h"

 moto_send*moto_can1;

static RC*rc;    

void can1_send(void *pvParameters)
{
	//获取电机输出信息指针
	
	moto_can1 = getmotosend();  
	rc = get_rc_data();
	
	while(1)
	{
		if( rc->sr == 1 ){
			if ( rc->L_y == -660 )             //拨到底部 发射 底盘测试 关闭云台
			{            
				TIM_SetCompare2(TIM1,1500);
				TIM_SetCompare1(TIM1,1500);	
		    Set_CloudMotor_Current(0,0);
		    Set_ChassisMotor_Current(moto_can1->standard,moto_can1->fire,0,0);
			}
			else
			{                                  //拨杆为1时，关闭输出
				TIM_SetCompare2(TIM1,0);
		    TIM_SetCompare1(TIM1,0);	
		    Set_CloudMotor_Current(0,0);
		    Set_ChassisMotor_Current(0,0,0,0);
			}
		}
		
		else if( rc->sr == 3 ) {            
			if ( rc->sl != 1 )            //拨到底部时，开启摩擦轮和拨弹轮
			{
		   TIM_SetCompare2(TIM1,1500);
		   TIM_SetCompare1(TIM1,1500);	
       Set_CloudMotor_Current(moto_can1->yaw,moto_can1->pitch);    //5100   3600
			 Set_ChassisMotor_Current(moto_can1->standard,moto_can1->fire,0,0);
			}				
			else                               //拨杆为3时云台扫描,关闭摩擦轮和拨弹轮
			{
			 TIM_SetCompare2(TIM1,0);
		   TIM_SetCompare1(TIM1,0);	
			 Set_CloudMotor_Current(moto_can1->yaw,moto_can1->pitch);
			 Set_ChassisMotor_Current(0,0,0,0);
			}
		}
		
		else if(  rc->sr == 2 ) {
		TIM_SetCompare2(TIM1,1500);
		TIM_SetCompare1(TIM1,1500);		
    Set_CloudMotor_Current(moto_can1->yaw,moto_can1->pitch);
		Set_ChassisMotor_Current(moto_can1->standard,moto_can1->fire,0,0);
		}
		
		else{                               //没接上遥控器时，关闭所有输出
		TIM_SetCompare2(TIM1,0);
		TIM_SetCompare1(TIM1,0);	
		Set_CloudMotor_Current(0,0);
		Set_ChassisMotor_Current(0,0,0,0);
		}
		  vTaskDelay(1);
	}
	
}

