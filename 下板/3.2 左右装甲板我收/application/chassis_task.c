#include "chassis_task.h"

static PID_INCR pid_standard;   //底盘电机pid参数初始化
static flag flag_cm1 = {0,0,0,0};   //标志位信息

static motor_date_ty*Encoder_standard; //底盘电机信息          
static RC*rc;                    //遥控器信
static button*button_key;       //按键信息
static encoder0*encoder_cm ;    //编码器信息
static motor_date_ty*fire_motor;
 
static moto_send moto_can = { 0, 0, 0, 0 };  //电机发送
static vision*vis;

static u8 rand_mode[1000] = {1,1,0,0,1,0,0,1,0,0,1,1,1,0,1,1,1,0,1,0,1,0,0,1,0,0,1,0,0,1,1,0,1,0,1,0,1,1,1,0,1,1,0,1,1,0,1,1,1,0,1,0,0,1,1,1,1,1,1,0,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,1,0,1,1,0,0,0,0,0,0,1,0,0,1,0,1,1,0,0,0,1,1,1,1,1,0,0,0,1,0,1,0,1,1,0,0,0,1,1,1,1,0,0,0,1,0,1,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,1,0,0,0,0,0,1,0,0,1,0,1,0,1,0,1,0,1,1,1,0,0,1,0,0,0,0,1,0,1,0,0,1,0,1,1,0,0,0,0,1,1,0,1,0,1,1,1,0,1,1,0,1,0,1,1,0,1,1,0,0,1,0,0,0,1,1,0,1,1,1,1,1,1,0,1,0,0,0,0,0,0,0,1,1,0,1,1,0,0,0,0,0,1,0,1,0,1,1,0,0,1,0,0,0,1,0,0,0,0,1,1,1,0,0,0,1,0,0,1,1,1,1,0,0,1,1,0,0,0,1,1,1,0,1,1,1,1,0,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,1,0,1,1,0,1,0,0,1,1,1,1,0,1,1,1,1,0,1,1,1,1,0,0,1,0,0,1,0,0,1,0,1,0,1,1,1,1,1,0,0,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,0,1,0,0,1,0,1,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,1,0,0,0,0,0,0,1,1,0,1,1,0,1,1,0,1,0,0,1,0,0,1,1,0,1,1,1,1,0,1,0,1,1,1,0,1,1,1,1,0,0,0,0,0,0,1,0,1,0,0,0,1,1,1,0,0,1,1,0,0,0,1,0,1,1,0,0,0,0,1,0,0,1,1,0,0,0,1,0,0,1,0,0,0,1,0,1,0,1,1,0,0,1,0,0,1,1,1,0,1,1,1,0,1,0,0,0,1,0,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,1,0,1,0,1,0,1,0,0,1,1,1,0,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,1,0,1,0,1,0,0,0,1,1,0,0,0,1,0,1,1,1,1,1,0,0,1,0,1,0,1,1,1,1,1,1,1,0,0,1,1,0,0,0,0,0,1,1,0,1,1,1,1,1,1,0,1,0,1,0,0,1,1,1,1,1,1,0,0,0,1,1,0,0,1,1,1,0,1,0,0,1,0,1,0,0,1,0,1,1,1,1,0,0,1,1,0,0,0,1,0,1,0,1,1,0,0,0,1,0,0,1,1,1,0,0,1,0,1,1,0,1,1,0,1,0,0,0,1,1,0,1,0,1,1,1,1,0,0,1,1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,0,1,0,1,0,0,1,0,1,0,0,0,1,1,0,1,1,1,0,1,0,1,1,0,0,0,1,1,1,0,0,0,0,1,1,1,1,0,0,1,0,0,1,0,1,1,1,0,0,0,1,0,1,1,1,0,1,0,1,0,0,0,1,1,0,0,0,1,0,1,1,1,1,1,0,1,1,1,0,1,1,0,1,1,1,1,1,1,0,0,0,0,1,0,0,0,0,0,1,1,0,0,0,1,0,1,1,0,1,0,1,1,0,0,1,1,1,0,0,1,0,0,1,1,1,1,1,0,0,1,0,0,1,1,0,0,0,0,0,0,1,1,0,1,0,0,1,1,1,0,0,1,0,0,0,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,0,1,1,0,0,0,0,1,0,1,0,1,0,0,0,1,1,1,0,0,0,0,0,0,1,1,0,1,0,1,1,0,1,1,0,0,1,0,0,0,1,1,1,0,1,0,1,1,1,1,1,0,0,1,1,0,1,0,0,1,0,1,0,0,1,1,1,1,1,1,1,0,0,1,0,1,1,1,1,0,1,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,1,0,1,0,0,1,1,0,1,0,1,1,0,0,0,0,1,1,0,1,1,1,1,1,0,1,0,1,0,0,1,1,0,0,0,1};
	//随机数
	
static judgement*jud_con_chass;

static model*task;

/*
函数内容：底盘任务
入口参数 无
返回值： 无
*/

void chassis_task (void *pvParameters)  
{ 
	//底盘3508pid初始化
  pid_incr_init(&pid_standard,25,0.15,0,15000,500);  //3508速度环
	
	//得到外部数据指针
	vis = get_vision_date();  
	rc = get_rc_data();         //遥控器
	Encoder_standard = getstandmotorpoint();   //底盘电机
	encoder_cm = get_encoder0_data();   //编码器数据
	task = gettasksend();
	fire_motor = getfiremotorpoint();   //发射拨弹轮信息
	jud_con_chass = getjudugementdate(); //
	
// 更改编码器地方	
	
	encoder_cm->max_echo = max_echo_guidao;
	
	while(1)
	{
		///////           初始化设置最大路径和起始点     ////// 
		standart_mode_rand();
		vTaskDelay(1);
		
		 if( rc->sr == 2 ){    //自动模式
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
//函数内容：底盘模式随机 函数 （随机生成速度和时间进行位移）
//入口参数 无
//返回值： 无
//*/


float kp_pass = 0.992f;


void standart_mode_rand(void)
{
	kp_pass = 0.992f; 
	
		speed_fast();          //弹丸数量不够
		
			//边缘碰撞
			if( (encoder_cm->all_echo > encoder_cm->max_echo - 1500 && flag_cm1.rand_speed > 0) || 
				 ( encoder_cm->all_echo <  1500 && flag_cm1.rand_speed <0 ))
				 {flag_cm1.rand_speed = - flag_cm1.rand_speed;
					kp_pass = 0.992f; 
				 }
		 
	    //对输出滤波	
				 
		  flag_cm1.set_speed = ( 1 - kp_pass ) * flag_cm1.rand_speed + kp_pass * flag_cm1.rand_last_speed;  
	    flag_cm1.rand_last_speed = flag_cm1.set_speed;
}

///*
//函数内容：底盘快速移动
//入口参数 无
//返回值： 无
//*/

void speed_fast(void)
{
	if( flag_cm1.rand_time_flag >= flag_cm1.rand_time )
		{
			if( ( rand() % 2) )
			flag_cm1.rand_speed = ( rand() % rand_speed_max_mid ) + rand_speed_mini_mid ;        //随机生成时间和速度
			else 
			flag_cm1.rand_speed = -(( rand() % rand_speed_max_mid ) + rand_speed_mini_mid ) ;
			
			flag_cm1.rand_time = rand() % rand_time_max + rand_time_mini;   //限制随机生成的速度和时间
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
//函数内容：返回电机输出值
//入口参数 无
//返回值： 无
//*/
moto_send*getmotosend(void)
{
	return &moto_can;
}

flag*getchassic_flag(void)
{
	
	return &flag_cm1;
}
