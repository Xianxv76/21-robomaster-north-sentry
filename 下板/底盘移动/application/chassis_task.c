#include "chassis_task.h"

static PID_INCR pid_standard;   //���̵��pid������ʼ��
 flag flag_cm1 = {0,0,0,0};   //��־λ��Ϣ

static motor_date_ty*Encoder_standard; //���̵����Ϣ          
static RC*rc;                    //ң������
static button*button_key;       //������Ϣ
static encoder0*encoder_cm ;    //��������Ϣ

static moto_send moto_can = { 0, 0, 0, 0 };  //�������

/*
�������ݣ���������
��ڲ��� ��
����ֵ�� ��
*/
float set_speed = 0;
float real_speed = 0;
float diff_speed = 0;

void chassis_task (void *pvParameters)  
{ 
	//����3508pid��ʼ��
  pid_incr_init(&pid_standard,25,0.15,0,15000,500);  //3508�ٶȻ�
	
	//�õ��ⲿ����ָ��
	button_key = getbutton();   //����
	rc = get_rc_data();         //ң����
	Encoder_standard = getstandmotorpoint();   //���̵��
	encoder_cm = get_encoder0_data();   //����������
	
	while(1)
	{
		///////           ��ʼ���������·������ʼ��     ////// 
		free_move();
		real_speed = Encoder_standard->ecd_rate/19.f;
		 if( rc->sr == 2 ){    //�Զ�ģʽ
		  //set_speed = flag_cm1.rand_speed;
		   moto_can.standard = pid_incr_calc(&pid_standard,( Encoder_standard->ecd_rate/19.f),set_speed);
		  // flag_cm1.rand_last_speed = flag_cm1.rand_speed; 
		 }
     else if (rc->sr ==1 || rc->sr ==3 ) 
		 {
			 if ( rc->L_y == -660 )
			 {			 
				 real_speed = Encoder_standard->ecd_rate/19.f;
				 diff_speed = set_speed - real_speed;
				 set_speed = rc->L_x * 2;
				 if( diff_speed > 500 )
				    moto_can.standard = 5000;
				 else if ( diff_speed < -500 )
					  moto_can.standard = -5000;
				 else 
					  moto_can.standard = pid_incr_calc(&pid_standard,( Encoder_standard->ecd_rate/19.f),set_speed);
			 }
			 else
		     moto_can.standard = 0;
		 }
     else
		   moto_can.standard = 0;
   vTaskDelay(1);	
   }
   }

/*
�������ݣ������˶��߼����Ⱥ���
��ڲ��� ��
����ֵ�� ��
*/	 
	 
	 void free_move(void)
	 {
		     //��ʼ��λ��  
						//δ����
					 if(flag_cm1.start == 0)         
					 {
						 int_location();	//��ʼ��λ��		 
						 //�������
							if ( flag_cm1.start == 1 )   
						{	
							if ( encoder_cm->max_echo <= 0)
							{
								while(1)
								{
									TIM_SetCompare3(TIM4,10000);
									vTaskDelay(300);
									TIM_SetCompare3(TIM4,0);
									vTaskDelay(300);
								}
							}
							///////////    ������ú��������һ��    //////////
							TIM_SetCompare3(TIM4,10000);
							 vTaskDelay(500);vTaskDelay(500);vTaskDelay(500);vTaskDelay(500);
							TIM_SetCompare3(TIM4,0);
							 ////��ʱ
							 vTaskDelay(500);vTaskDelay(500);vTaskDelay(500);vTaskDelay(500);
						}}
					 else
					 {
						 standart_mode_rand();
					 }
					   //standart_mode_1();																	 
           vTaskDelay(1);
	 }

	 
/*
�������ݣ���ʼ��λ�ú���
��ڲ��� ��
����ֵ�� ��
*/


void int_location(void)
{
	if( button_key->flag == 1 && button_key->get == 1 )
	{
		//��ʼ��λ��
		encoder_cm->max_echo = encoder_cm->all_echo ;
	//encoder_cm->all_echo = 0;
		BLUE = 0;
		RED = 1;
	}
  if( button_key->flag == 2 && button_key->get == 1 )
	{
		//��ʼ��λ��
  //encoder_cm->max_echo = encoder_cm->all_echo ;
		encoder_cm->max_echo = encoder_cm->max_echo - encoder_cm->all_echo;
	  encoder_cm->all_echo = 0;
	  RED = 0;
		GREEN =1;
		flag_cm1.start = 1;
	}
}

///*
//�������ݣ�����ģʽ��� ���� ����������ٶȺ�ʱ�����λ�ƣ�
//��ڲ��� ��
//����ֵ�� ��
//*/


float kp_pass = 0.99f;


void standart_mode_rand(void)
{
	if( flag_cm1.rand_time_flag >= flag_cm1.rand_time )
		{	
			if( ( rand() % 2) )
			flag_cm1.rand_speed = ( rand() % rand_speed_max_mid ) + rand_speed_mini_mid ;        //�������ʱ����ٶ�
			else 
			flag_cm1.rand_speed = -(( rand() % rand_speed_max_mid ) + rand_speed_mini_mid ) ;

			flag_cm1.rand_time = rand() % rand_time_max + rand_time_mini;   //����������ɵ��ٶȺ�ʱ��
			
			flag_cm1.rand_time_flag = 0;
			kp_pass = 0.99f;
		}
	else {
		    flag_cm1.rand_time_flag ++;
		    kp_pass = 0.99f;
	     } 

	//��Ե��ײ
	if( (encoder_cm->all_echo > encoder_cm->max_echo - 1500 && flag_cm1.rand_speed > 0) || 
		 ( encoder_cm->all_echo <  1500 && flag_cm1.rand_speed <0 ))
		 {flag_cm1.rand_speed = - flag_cm1.rand_speed;
			kp_pass = 0.9f; 
		 }
		 
	    //������˲�
    	set_speed = ( 1 - kp_pass ) * flag_cm1.rand_speed + kp_pass * flag_cm1.rand_last_speed;  
	    flag_cm1.rand_last_speed = set_speed;
}


///*
//�������ݣ����ص�����ֵ
//��ڲ��� ��
//����ֵ�� ��
//*/
moto_send*getmotosend(void)
{
	return &moto_can;
}


