#include "fire_task.h"

static PID pid_2006;   //���pid������ʼ��
static motor_date_ty*fire_motor;

float setspeed = 0;

static RC*rc;
static vision*vis;
static moto_send*moto_can;
	flag_fire flag_;
	judgement*judge;
	
/*
�������ݣ���������
��ڲ��� ��
����ֵ�� ��
*/


void fire_task (void *pvParameters)  
{  
	//�����Ϣ	
	  //pid_init(&pid_2006,17,0.015,0,7000,1000,1000,9000);  //2006
	  pid_init(&pid_2006,20,0,0,7000,0,0,10000);
	  fire_motor = getfiremotorpoint();
	  rc = get_rc_data();
	  vis = get_vision_date();
	  moto_can = getmotosend();
  	fire_motor = getfiremotorpoint();
	while(1)
	{
	if( rc->sr == 2  ){
		if( ( fire_motor->round_cnt/36.f ) <= 75 ) //�ж��Ƿ��е�
			{
				if( vis->command == 1  )    //ʶ��Ŀ�꣬��
				{	
          //if ( judge->shoot_heat <= 260 )					
					  setspeed = 400;
					//else
						//setspeed = 400;
				}
				else 
					setspeed = 0;
			}
		else
			 setspeed = 0;

		moto_can->fire  =	Calculate_Current_Value( &pid_2006,setspeed,fire_motor->ecd_rate/36.f);    //���䲿��
	}
	
	else if( rc->sr == 3 )  //������Դ�
   {
		if ( rc->L_y == -660 )  	
		{
		 if( vis->command == 1  )    //ʶ��Ŀ�꣬��		
		 {
			 //if ( judge->shoot_heat <= 260 )					
					  setspeed = 400;
			 //else
						//setspeed = 400;
		}
		 else 
		  setspeed = 0;
	  }
		else
			moto_can->fire = 0;
		
		moto_can->fire  =	Calculate_Current_Value( &pid_2006,setspeed,fire_motor->ecd_rate/36.f);    //���䲿��
	  } 
  else if( rc->sr == 1 )                   //���Դ�
	{
		if ( rc->L_y == -660 )  	
		 setspeed = rc->R_x;
		else 
		 setspeed = 0;
		moto_can->fire  =	Calculate_Current_Value( &pid_2006,setspeed,fire_motor->ecd_rate/36.f);
	}
	
    vTaskDelay(1);	   
   }
	}








