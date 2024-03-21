#include "model_task.h"

 static PID_INCR pid_6020_yaw;   //6020�ٶȻ�
 static PID      pid_yaw_cirle;   //6020�ǶȻ�

 static PID  pid_6020_pitch;   //6020�ٶȻ�
 static PID       pid_pitch_cirle;   //6020�ǶȻ� 

 static moto_send*moto_can;     //can ����
 static vision*vis;
 
 static mpu*bmi;
 
 
 
/*
�������ݣ���������
��ڲ��� ��
����ֵ�� ��
*/
	

static model task;

float look_moto_yaw = 0;

int ccc = 0;

void model_task(void *pvParameters)  
{ 
	 
	  //yaw   ��������
	  pid_incr_init(&pid_6020_yaw,175,0.9,0,30000,5000);  //6020�ٶȻ�
	  pid_init(&pid_yaw_cirle,25,0,0,200,50,50,200);  //6020�ǶȻ�
	
	  //pitch  
	  pid_init(&pid_6020_pitch,230,5,0,30000,5000,10000,30000);  //6020�ٶȻ�
	  pid_init(&pid_pitch_cirle,20,0,0,200,50,100,200);  //6020�ǶȻ�
	
	  //�õ�����ָ���ַ����Ϣ��ȡ
		vis = get_vision_date();                              //�Ӿ���Ϣ
	  moto_can = getmotosend();                             //can��������   
	  bmi = mpu_get_data();                                 //mpu��������
	
  
	while(1)
			{
				task.time_flag++;
				if( task.time_flag >= ins_zero_flag + 2000 )  
				{  
							 
					
							 //task.setx = rc->R_x ;           //�����ײ�ʱ������Ħ���ֺͲ�����
							 //task.setx += rc->R_x/1000.f;   
							 //task.setx = 0 ;
							 //task.sety = -rc->R_y ;  //ң����������̨ 
						 
							 //task.sety -= rc->R_y/1000.f;

							 //task.setx =  0;
							 //task.setx = vis->yaw_offset ;   
							 //task.sety = vis->pitch_offset;  // ����
					     bmi_bee();  //����������
					
							 task.speed_y = bmi->gyro_buf[1] *57.3f;
							 task.speed_x = bmi->gyro_buf[2] *57.3f;	  //yaw���ٶ�

							/////       ʶ��Ŀ��ʱ����
							if ( vis->command == 1 )
									 vis_get();  //����
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


	
	
	
	
//����������
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





///ʶ�𵽺��Ӿ�����

void vis_get(void)
{
	 task.setx = vis->set_yaw.angle*0.2f + bmi->yaw;
	 task.sety = vis->set_pitch.angle*0.2f +bmi->pit;          //pid ����
	 task.sety  = task.sety  < -4 ? -4 : task.sety ;
	 task.sety  = task.sety  >  50 ?  50 : task.sety ;  //��̨pitch��λ	
		
	 /////////////////               yaw          /////////////

	 moto_can->yaw_cirle =	Calculate_Current_Value( &pid_yaw_cirle, task.setx , bmi->yaw );
	 moto_can->yaw  =	pid_incr_calc( &pid_6020_yaw,task.speed_x,moto_can->yaw_cirle );
		
	 ////////////////                pitch       ///////////    5100   3600
	 moto_can->pitch_cirle = Calculate_Current_Value( &pid_pitch_cirle, task.sety , bmi->pit ); //�ǶȻ�  
	 moto_can->pitch  =	-Calculate_Current_Value( &pid_6020_pitch, moto_can->pitch_cirle, task.speed_y ); //�ٶȻ�
		
	 task.vis_time = 0;
}








//δʶ��Ŀ��Ѳ��ģʽ

void vis_cruise(void)
{
		 task.vis_time++;
					 if( task.vis_time >= 800 )
					 {
						 //yaw��Ѳ��
					  moto_can->yaw  =	pid_incr_calc( &pid_6020_yaw,task.speed_x,-120 );
						//pitch��Ѳ��						
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
			      task.sety  = task.sety  >  50 ?  50 : task.sety ;  //��̨pitch��λ										 
	          moto_can->pitch_cirle = Calculate_Current_Value( &pid_pitch_cirle, task.sety , bmi->pit ); //�ǶȻ�  
					  moto_can->pitch  =	-Calculate_Current_Value( &pid_6020_pitch, moto_can->pitch_cirle, task.speed_y ); //�ٶȻ�							
						}		
}




