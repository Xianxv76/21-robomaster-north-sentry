#include "model_task.h"

 static PID_INCR pid_6020_yaw;   //6020速度环
 static PID      pid_yaw_cirle;   //6020角度环

 static PID  pid_6020_pitch;   //6020速度环
 static PID       pid_pitch_cirle;   //6020角度环 

 static moto_send*moto_can;     //can 发送
 static vision*vis;
 
 static mpu*bmi;
 
 
 
/*
函数内容：底盘任务
入口参数 无
返回值： 无
*/
	

static model task;

float look_moto_yaw = 0;

int ccc = 0;

void model_task(void *pvParameters)  
{ 
	 
	  //yaw   整定完事
	  pid_incr_init(&pid_6020_yaw,175,0.9,0,30000,5000);  //6020速度环
	  pid_init(&pid_yaw_cirle,25,0,0,200,50,50,200);  //6020角度环
	
	  //pitch  
	  pid_init(&pid_6020_pitch,230,5,0,30000,5000,10000,30000);  //6020速度环
	  pid_init(&pid_pitch_cirle,20,0,0,200,50,100,200);  //6020角度环
	
	  //得到各处指针地址，信息获取
		vis = get_vision_date();                              //视觉信息
	  moto_can = getmotosend();                             //can发送数据   
	  bmi = mpu_get_data();                                 //mpu部分数据
	
  
	while(1)
			{
				task.time_flag++;
				if( task.time_flag >= ins_zero_flag + 2000 )  
				{  
							 
					
							 //task.setx = rc->R_x ;           //拨到底部时，开启摩擦轮和拨弹轮
							 //task.setx += rc->R_x/1000.f;   
							 //task.setx = 0 ;
							 //task.sety = -rc->R_y ;  //遥控器调试云台 
						 
							 //task.sety -= rc->R_y/1000.f;

							 //task.setx =  0;
							 //task.setx = vis->yaw_offset ;   
							 //task.sety = vis->pitch_offset;  // 自瞄
					     bmi_bee();  //蜂鸣器函数
					
							 task.speed_y = bmi->gyro_buf[1] *57.3f;
							 task.speed_x = bmi->gyro_buf[2] *57.3f;	  //yaw轴速度

							/////       识别到目标时自瞄
							if ( vis->command == 1 )
									 vis_get();  //自瞄
							else
									 vis_cruise();
							
				}
			else 
				{
					moto_can->yaw = 0;
					moto_can->pitch = 0;
					TIM_SetCompare3(TIM4,0);
				}
				
				vTaskDelay(1);	
				}
  }	


	
	
	
	
//蜂鸣器鸣叫
void bmi_bee(void)
{
	if( task.bee_flag ==0 )
		{
				 TIM_SetCompare3(TIM4,10000);
				vTaskDelay(500);vTaskDelay(500);vTaskDelay(500);
				task.bee_flag = 1;
				TIM_SetCompare3(TIM4,0);
		}
}





///识别到后视觉自瞄

void vis_get(void)
{
	 task.setx = vis->set_yaw.angle*0.2f + bmi->yaw;
	 task.sety = vis->set_pitch.angle*0.2f +bmi->pit;          //pid 自瞄
	 task.sety  = task.sety  < -4 ? -4 : task.sety ;
	 task.sety  = task.sety  >  50 ?  50 : task.sety ;  //云台pitch限位	
		
	 /////////////////               yaw          /////////////

	 moto_can->yaw_cirle =	Calculate_Current_Value( &pid_yaw_cirle, task.setx , bmi->yaw );
	 moto_can->yaw  =	pid_incr_calc( &pid_6020_yaw,task.speed_x,moto_can->yaw_cirle );
		
	 ////////////////                pitch       ///////////    5100   3600
	 moto_can->pitch_cirle = Calculate_Current_Value( &pid_pitch_cirle, task.sety , bmi->pit ); //角度环  
	 moto_can->pitch  =	-Calculate_Current_Value( &pid_6020_pitch, moto_can->pitch_cirle, task.speed_y ); //速度环
		
	 task.vis_time = 0;
}








//未识别到目标巡航模式

void vis_cruise(void)
{
		 task.vis_time++;
					 if( task.vis_time >= 800 )
					 {
						 //yaw轴巡航
					  moto_can->yaw  =	pid_incr_calc( &pid_6020_yaw,task.speed_x,-120 );
						//pitch轴巡航						
									 if( task.flag_pit == 0){
										if( bmi->pit  < 0 )
										{
											 task.flag_pit = 1; 
										   
										}
										else
											  task.sety = task.sety - 0.06f;
									  }
									
							    else{
									 if( bmi->pit  >  45  )
									 {
										 task.flag_pit = 0;		
									 }										 
									 else
									     task.sety = task.sety + 0.06f; 
								   }
						
				 	  task.sety  = task.sety  < -4 ? -4 : task.sety ;
			      task.sety  = task.sety  >  50 ?  50 : task.sety ;  //云台pitch限位										 
	          moto_can->pitch_cirle = Calculate_Current_Value( &pid_pitch_cirle, task.sety , bmi->pit ); //角度环  
					  moto_can->pitch  =	-Calculate_Current_Value( &pid_6020_pitch, moto_can->pitch_cirle, task.speed_y ); //速度环							
						}		
}




