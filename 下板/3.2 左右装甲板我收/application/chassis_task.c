#include "chassis_task.h"

static PID_INCR pid_standard;   //���̵��pid������ʼ��
static flag flag_cm1 = {0,0,0,0};   //��־λ��Ϣ

static motor_date_ty*Encoder_standard; //���̵����Ϣ          
static RC*rc;                    //ң������
static button*button_key;       //������Ϣ
static encoder0*encoder_cm ;    //��������Ϣ
static motor_date_ty*fire_motor;
 
static moto_send moto_can = { 0, 0, 0, 0 };  //�������
static vision*vis;

static u8 rand_mode[1000] = {1,1,0,0,1,0,0,1,0,0,1,1,1,0,1,1,1,0,1,0,1,0,0,1,0,0,1,0,0,1,1,0,1,0,1,0,1,1,1,0,1,1,0,1,1,0,1,1,1,0,1,0,0,1,1,1,1,1,1,0,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,1,0,1,1,0,0,0,0,0,0,1,0,0,1,0,1,1,0,0,0,1,1,1,1,1,0,0,0,1,0,1,0,1,1,0,0,0,1,1,1,1,0,0,0,1,0,1,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,1,0,0,0,0,0,1,0,0,1,0,1,0,1,0,1,0,1,1,1,0,0,1,0,0,0,0,1,0,1,0,0,1,0,1,1,0,0,0,0,1,1,0,1,0,1,1,1,0,1,1,0,1,0,1,1,0,1,1,0,0,1,0,0,0,1,1,0,1,1,1,1,1,1,0,1,0,0,0,0,0,0,0,1,1,0,1,1,0,0,0,0,0,1,0,1,0,1,1,0,0,1,0,0,0,1,0,0,0,0,1,1,1,0,0,0,1,0,0,1,1,1,1,0,0,1,1,0,0,0,1,1,1,0,1,1,1,1,0,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,1,0,1,1,0,1,0,0,1,1,1,1,0,1,1,1,1,0,1,1,1,1,0,0,1,0,0,1,0,0,1,0,1,0,1,1,1,1,1,0,0,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,0,1,0,0,1,0,1,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,1,0,0,0,0,0,0,1,1,0,1,1,0,1,1,0,1,0,0,1,0,0,1,1,0,1,1,1,1,0,1,0,1,1,1,0,1,1,1,1,0,0,0,0,0,0,1,0,1,0,0,0,1,1,1,0,0,1,1,0,0,0,1,0,1,1,0,0,0,0,1,0,0,1,1,0,0,0,1,0,0,1,0,0,0,1,0,1,0,1,1,0,0,1,0,0,1,1,1,0,1,1,1,0,1,0,0,0,1,0,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,1,0,1,0,1,0,1,0,0,1,1,1,0,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,1,0,1,0,1,0,0,0,1,1,0,0,0,1,0,1,1,1,1,1,0,0,1,0,1,0,1,1,1,1,1,1,1,0,0,1,1,0,0,0,0,0,1,1,0,1,1,1,1,1,1,0,1,0,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,1,1,1,0,1,0,0,1,0,1,0,0,1,0,1,1,1,1,0,0,1,1,0,0,0,1,0,1,0,1,1,0,0,0,1,0,0,1,1,1,0,0,1,0,1,1,0,1,1,0,1,0,0,0,1,1,0,1,0,1,1,1,1,0,0,1,1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,0,1,0,1,0,0,1,0,1,0,0,0,1,1,0,1,1,1,0,1,0,1,1,0,0,0,1,1,1,0,0,0,0,1,1,1,1,0,0,1,0,0,1,0,1,1,1,0,0,0,1,0,1,1,1,0,1,0,1,0,0,0,1,1,0,0,0,1,0,1,1,1,1,1,0,1,1,1,0,1,1,0,1,1,1,1,1,1,0,0,0,0,1,0,0,0,0,0,1,1,0,0,0,1,0,1,1,0,1,0,1,1,0,0,1,1,1,0,0,1,0,0,1,1,1,1,1,0,0,1,0,0,1,1,0,0,0,0,0,0,1,1,0,1,0,0,1,1,1,0,0,1,0,0,0,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,0,1,1,0,0,0,0,1,0,1,0,1,0,0,0,1,1,1,0,0,0,0,0,0,1,1,0,1,0,1,1,0,1,1,0,0,1,0,0,0,1,1,1,0,1,0,1,1,1,1,1,0,0,1,1,0,1,0,0,1,0,1,0,0,1,1,1,1,1,1,1,0,0,1,0,1,1,1,1,0,1,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,1,0,1,0,0,1,1,0,1,0,1,1,0,0,0,0,1,1,0,1,1,1,1,1,0,1,0,1,0,0,1,1,0,0,0,1};
	//�����
	
static judgement*jud_con_chass;

static model*task;

/*
�������ݣ���������
��ڲ��� ��
����ֵ�� ��
*/

void chassis_task (void *pvParameters)  
{ 
	//����3508pid��ʼ��
  pid_incr_init(&pid_standard,25,0.15,0,15000,500);  //3508�ٶȻ�
	
	//�õ��ⲿ����ָ��
	vis = get_vision_date();  
	rc = get_rc_data();         //ң����
	Encoder_standard = getstandmotorpoint();   //���̵��
	encoder_cm = get_encoder0_data();   //����������
	task = gettasksend();
	fire_motor = getfiremotorpoint();   //���䲦������Ϣ
	jud_con_chass = getjudugementdate(); //
	
// ���ı������ط�	
	
	encoder_cm->max_echo = max_echo_guidao;
	
	while(1)
	{
		///////           ��ʼ���������·������ʼ��     ////// 
		standart_mode_rand();
		vTaskDelay(1);
		
		 if( rc->sr == 2 ){    //�Զ�ģʽ
		   moto_can.standard = pid_incr_calc(&pid_standard,( Encoder_standard->ecd_rate/19.f),flag_cm1.set_speed);
		 }
		 
		 
     else if (rc->sr ==1 || rc->sr ==3 ) 
		 {
			 if ( rc->L_y == -660 )
			 {			 
				 flag_cm1.set_speed = rc->L_x * 2;
				 
					  moto_can.standard = pid_incr_calc(&pid_standard,( Encoder_standard->ecd_rate/19.f),flag_cm1.set_speed);
			 }
			 else
		     moto_can.standard = 0;
		 }
     else
		   moto_can.standard = 0;
   vTaskDelay(1);	
   }
   }


///*
//�������ݣ�����ģʽ��� ���� ����������ٶȺ�ʱ�����λ�ƣ�
//��ڲ��� ��
//����ֵ�� ��
//*/


float kp_pass = 0.992f;


void standart_mode_rand(void)
{
	kp_pass = 0.992f; 
	
		speed_fast();          //������������
		
			//��Ե��ײ
			if( (encoder_cm->all_echo > encoder_cm->max_echo - 1500 && flag_cm1.rand_speed > 0) || 
				 ( encoder_cm->all_echo <  1500 && flag_cm1.rand_speed <0 ))
				 {flag_cm1.rand_speed = - flag_cm1.rand_speed;
					kp_pass = 0.992f; 
				 }
		 
	    //������˲�	
				 
		  flag_cm1.set_speed = ( 1 - kp_pass ) * flag_cm1.rand_speed + kp_pass * flag_cm1.rand_last_speed;  
	    flag_cm1.rand_last_speed = flag_cm1.set_speed;
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


int needle = 0;

	
void speed_low(void)
{
	static int flag = 0;
	if( jud_con_chass->admier_id != 0 )
		{
			if( flag == 0 )
			{
					if(  rand_mode[needle] ) 
						flag_cm1.rand_speed = 2000;
					else
						flag_cm1.rand_speed = -2000;
					
					needle = (needle +1)%1000;
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
