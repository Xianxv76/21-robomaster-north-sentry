#include "can1_send.h"

 moto_send*moto_can1;

static RC*rc;

static motor_date_ty*Cloud_pitch;      

int aa = 0;

void can1_send(void *pvParameters)
{
	//��ȡ��������Ϣָ��
	
	moto_can1 = getmotosend();
	Cloud_pitch = get6020pitchmotorpoint();   
	rc = get_rc_data();
	
	while(1)
	{
		aa++;
		
		if( rc->sr == 1 ){
			if ( rc->L_y == -660 )             //�����ײ� ���� ���̲��� �ر���̨
			{            
				TIM_SetCompare2(TIM1,1400);
				TIM_SetCompare1(TIM1,1400);	
		    Set_CloudMotor_Current(0,0);
		    Set_ChassisMotor_Current(moto_can1->standard,moto_can1->fire,0,0);
			}
			else
			{                                  //����Ϊ1ʱ���ر����
				TIM_SetCompare2(TIM1,0);
		    TIM_SetCompare1(TIM1,0);	
		    Set_CloudMotor_Current(0,0);
		    Set_ChassisMotor_Current(0,0,0,0);
			}
		}
		
		else if( rc->sr == 3 ) {            
			if ( rc->L_y == -660 )            //�����ײ�ʱ������Ħ���ֺͲ�����
			{
		   TIM_SetCompare2(TIM1,0);
		   TIM_SetCompare1(TIM1,0);	
//				TIM_SetCompare2(TIM1,0);
//		   TIM_SetCompare1(TIM1,0);	
       Set_CloudMotor_Current(0,0); 
				Set_ChassisMotor_Current(0,0,0,0);
//       Set_CloudMotor_Current(moto_can1->yaw,moto_can1->pitch);    //5100   3600
//			 Set_ChassisMotor_Current(moto_can1->standard,moto_can1->fire,0,0);
			}				
			else                               //����Ϊ3ʱ��̨ɨ��,�ر�Ħ���ֺͲ�����
			{
			 TIM_SetCompare2(TIM1,0);
		   TIM_SetCompare1(TIM1,0);	
//			 Set_CloudMotor_Current(moto_can1->yaw,moto_can1->pitch);
				Set_CloudMotor_Current(0,0); 
			 Set_ChassisMotor_Current(0,0,0,0);
			}
		}
		
		else if(  rc->sr == 2 ) {
		//TIM_SetCompare2(TIM1,1500);
		//TIM_SetCompare1(TIM1,1500);		
		//Set_CloudMotor_Current(moto_can1->yaw,moto_can1->pitch);
		TIM_SetCompare2(TIM1,0);
		TIM_SetCompare1(TIM1,0);	
		Set_CloudMotor_Current(0,0);	
    Set_ChassisMotor_Current(0,0,0,0);
		//Set_ChassisMotor_Current(moto_can1->standard,moto_can1->fire,0,0);
		}
		
		else{                               //û����ң����ʱ���ر��������
		TIM_SetCompare2(TIM1,0);
		TIM_SetCompare1(TIM1,0);	
		Set_CloudMotor_Current(0,0);
		Set_ChassisMotor_Current(0,0,0,0);
		}
		  vTaskDelay(1);
	}
	
}

