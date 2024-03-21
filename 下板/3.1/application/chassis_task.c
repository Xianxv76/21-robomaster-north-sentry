#include "chassis_task.h"

static PID_INCR pid_standard;   //���̵��pid������ʼ��
 flag flag_cm1 = {0,0,0,0};   //��־λ��Ϣ

static motor_date_ty*Encoder_standard; //���̵����Ϣ          
static RC*rc;                    //ң������
static button*button_key;       //������Ϣ
static encoder0*encoder_cm ;    //��������Ϣ
static motor_date_ty*fire_motor;
 
static moto_send moto_can = { 0, 0, 0, 0 };  //�������
 static vision*vis;

u8 rand_mode[1000] = {1,1,0,0,1,0,0,1,0,0,1,1,1,0,1,1,1,0,1,0,1,0,0,1,0,0,1,0,0,1,1,0,1,0,1,0,1,1,1,0,1,1,0,1,1,0,1,1,1,0,1,0,0,1,1,1,1,1,1,0,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,1,0,1,1,0,0,0,0,0,0,1,0,0,1,0,1,1,0,0,0,1,1,1,1,1,0,0,0,1,0,1,0,1,1,0,0,0,1,1,1,1,0,0,0,1,0,1,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,1,0,0,0,0,0,1,0,0,1,0,1,0,1,0,1,0,1,1,1,0,0,1,0,0,0,0,1,0,1,0,0,1,0,1,1,0,0,0,0,1,1,0,1,0,1,1,1,0,1,1,0,1,0,1,1,0,1,1,0,0,1,0,0,0,1,1,0,1,1,1,1,1,1,0,1,0,0,0,0,0,0,0,1,1,0,1,1,0,0,0,0,0,1,0,1,0,1,1,0,0,1,0,0,0,1,0,0,0,0,1,1,1,0,0,0,1,0,0,1,1,1,1,0,0,1,1,0,0,0,1,1,1,0,1,1,1,1,0,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,1,0,1,1,0,1,0,0,1,1,1,1,0,1,1,1,1,0,1,1,1,1,0,0,1,0,0,1,0,0,1,0,1,0,1,1,1,1,1,0,0,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,0,1,0,0,1,0,1,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,1,0,0,0,0,0,0,1,1,0,1,1,0,1,1,0,1,0,0,1,0,0,1,1,0,1,1,1,1,0,1,0,1,1,1,0,1,1,1,1,0,0,0,0,0,0,1,0,1,0,0,0,1,1,1,0,0,1,1,0,0,0,1,0,1,1,0,0,0,0,1,0,0,1,1,0,0,0,1,0,0,1,0,0,0,1,0,1,0,1,1,0,0,1,0,0,1,1,1,0,1,1,1,0,1,0,0,0,1,0,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,1,0,1,0,1,0,1,0,0,1,1,1,0,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,1,0,1,0,1,0,0,0,1,1,0,0,0,1,0,1,1,1,1,1,0,0,1,0,1,0,1,1,1,1,1,1,1,0,0,1,1,0,0,0,0,0,1,1,0,1,1,1,1,1,1,0,1,0,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,1,1,1,0,1,0,0,1,0,1,0,0,1,0,1,1,1,1,0,0,1,1,0,0,0,1,0,1,0,1,1,0,0,0,1,0,0,1,1,1,0,0,1,0,1,1,0,1,1,0,1,0,0,0,1,1,0,1,0,1,1,1,1,0,0,1,1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,0,1,0,1,0,0,1,0,1,0,0,0,1,1,0,1,1,1,0,1,0,1,1,0,0,0,1,1,1,0,0,0,0,1,1,1,1,0,0,1,0,0,1,0,1,1,1,0,0,0,1,0,1,1,1,0,1,0,1,0,0,0,1,1,0,0,0,1,0,1,1,1,1,1,0,1,1,1,0,1,1,0,1,1,1,1,1,1,0,0,0,0,1,0,0,0,0,0,1,1,0,0,0,1,0,1,1,0,1,0,1,1,0,0,1,1,1,0,0,1,0,0,1,1,1,1,1,0,0,1,0,0,1,1,0,0,0,0,0,0,1,1,0,1,0,0,1,1,1,0,0,1,0,0,0,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,0,1,1,0,0,0,0,1,0,1,0,1,0,0,0,1,1,1,0,0,0,0,0,0,1,1,0,1,0,1,1,0,1,1,0,0,1,0,0,0,1,1,1,0,1,0,1,1,1,1,1,0,0,1,1,0,1,0,0,1,0,1,0,0,1,1,1,1,1,1,1,0,0,1,0,1,1,1,1,0,1,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,1,0,1,0,0,1,1,0,1,0,1,1,0,0,0,0,1,1,0,1,1,1,1,1,0,1,0,1,0,0,1,1,0,0,0,1};
	//�����
	
judgement*jud_con_chass;

 model*task;

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
	vis = get_vision_date();  
	rc = get_rc_data();         //ң����
	Encoder_standard = getstandmotorpoint();   //���̵��
	encoder_cm = get_encoder0_data();   //����������
	task = gettasksend();
	fire_motor = getfiremotorpoint();   //���䲦������Ϣ
	jud_con_chass = getjudugementdate(); //
	
// ���ı������ط�	
	
	encoder_cm->max_echo = 8000;
	while(1)
	{
		///////           ��ʼ���������·������ʼ��     ////// 
		//free_move();  //�Զ�ģʽ����
		standart_mode_rand();
		vTaskDelay(1);
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
									flag_cm1.bee_flag =1;
									TIM_SetCompare3(TIM4,10000);
									vTaskDelay(300);
									TIM_SetCompare3(TIM4,0);
									vTaskDelay(300);
								}
							}
							///////////    ������ú��������һ��    //////////
							flag_cm1.bee_flag =1;
							TIM_SetCompare3(TIM4,10000);
							 vTaskDelay(500);vTaskDelay(500);vTaskDelay(500);vTaskDelay(500);
							TIM_SetCompare3(TIM4,0);
							 ////��ʱ
							 vTaskDelay(500);vTaskDelay(500);vTaskDelay(500);vTaskDelay(500);
							flag_cm1.bee_flag =0;
						}}
					 else
					 {
						 standart_mode_rand();   //����˶�
					 }																 
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


float kp_pass = 0.992f;

float shot = 0;

float time = 0;
	
int ccc = 0;

void standart_mode_rand(void)
{
	kp_pass = 0.992f; 
	shot = fire_motor->round_cnt/36.f ;
	
	if( shot <= 75 )           //����������
	{
		if( vis->command == 0 )  //δʶ��
		{
				time ++;
			if( time>=1500 )
			  speed_fast();       
			else
				speed_low();
		}
		else                     //ʶ��
		{
			time = 0;
			speed_low();
		}
	}
	else
		speed_fast();          //������������
		

			//��Ե��ײ
			if( (encoder_cm->all_echo > encoder_cm->max_echo - 1500 && flag_cm1.rand_speed > 0) || 
				 ( encoder_cm->all_echo <  1500 && flag_cm1.rand_speed <0 ))
				 {flag_cm1.rand_speed = - flag_cm1.rand_speed;
					kp_pass = 0.992f; 
				 }
		 
	    //������˲�
    	
				 
		  set_speed = ( 1 - kp_pass ) * flag_cm1.rand_speed + kp_pass * flag_cm1.rand_last_speed;  
	    flag_cm1.rand_last_speed = set_speed;
}

///*
//�������ݣ����̿����ƶ�
//��ڲ��� ��
//����ֵ�� ��
//*/

void speed_fast(void)
{
	if( flag_cm1.rand_time_flag >= flag_cm1.rand_time )
		{
			if( ( rand() % 2) )
			flag_cm1.rand_speed = ( rand() % rand_speed_max_mid ) + rand_speed_mini_mid ;        //�������ʱ����ٶ�
			else 
			flag_cm1.rand_speed = -(( rand() % rand_speed_max_mid ) + rand_speed_mini_mid ) ;
			
			flag_cm1.rand_time = rand() % rand_time_max + rand_time_mini;   //����������ɵ��ٶȺ�ʱ��
			flag_cm1.rand_time_flag = 0;
		}
		else
		  flag_cm1.rand_time_flag ++;
		
}


int aaa = 0;

	
void speed_low(void)
{
	static int flag = 0;
	if( jud_con_chass->admier_id != 0 )
		{
			if( flag == 0 )
			{
					if(  rand_mode[aaa] ) 
						flag_cm1.rand_speed = 2000;
					else
						flag_cm1.rand_speed = -2000;
					
					aaa = (aaa +1)%1000;
					
					flag = 1;
			}
		}
	else
	{
		flag = 0;
		flag_cm1.rand_speed = 0;
	}
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

flag*getchassic_flag(void)
{
	
	return &flag_cm1;
}
