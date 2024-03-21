#include "model_task.h"

#define NOW 0
#define LAST 1
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2

 static PID_INCR pid_6020_yaw;   //6020速度环
 static PID      pid_yaw_cirle;   //6020角度环

 static PID  pid_6020_pitch;   //6020速度环
 static PID       pid_pitch_cirle;   //6020角度环 

 static moto_send*moto_can;     //can 发送
 static vision*vis;
 
 static mpu*bmi;
 
 static flag*chassic;
int vision_time_js;

uint32_t Vision_Time[2];// NOW/LAST

float error_yaw_k   = 1;
float error_pitch_k = 1;

float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//卡尔曼滤波速度测量值
float *yaw_kf_result, *pitch_kf_result;//二阶卡尔曼滤波结果,0角度 1速度
float pitch_kf_result_0, yaw_kf_result_1;

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;
speed_calc_data_t Vision_Yaw_speed_Struct;
speed_calc_data_t Vision_Pitch_speed_Struct;

uint32_t Vision_Time[2];// NOW/LAST

kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.001/*0.002*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {500, 0, 0, 1000}//500 1000
};//初始化yaw的部分kalman参数

kalman_filter_init_t pitch_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.001/*0.002*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}
};//初始化pitch的部分kalman参数


float debug_y_sk;// = 38;//35;//30;//移动预测系数,越大预测越多
float debug_y_sb_sk;//哨兵预测系数
float debug_y_sb_brig_sk;//桥头哨兵
float debug_p_sk;//移动预测系数,越大预测越多
float debug_auto_err_y;// = 10;//15;//10;//15;//yaw角度过大关闭预测
float debug_auto_err_p;//pitch角度过大关闭预测
float debug_kf_delay;// = 150;//100;//200;//120;//150;//预测延时开启
float debug_kf_speed_yl;//yaw速度过低关闭预测
float debug_kf_speed_yl_sb;//抬头打哨兵时减小最低可开预测量
float debug_kf_speed_yh;//yaw速度过高关闭预测
float debug_kf_speed_pl;//pitch速度过低关闭预测
float debug_kf_y_angcon;// = 130;//125;//115;//135;//yaw预测量限幅
float debug_kf_p_angcon;//pitch预测量限幅
 

/*一阶卡尔曼*/
//云台角度误差卡尔曼
extKalman_t Gimbal_Pitch_Mech_Error_Kalman;//定义一个kalman指针
extKalman_t Gimbal_Pitch_Gyro_Error_Kalman;//定义一个kalman指针
extKalman_t Gimbal_Yaw_Mech_Error_Kalman;//定义一个kalman指针
extKalman_t Gimbal_Yaw_Gyro_Error_Kalman;//定义一个kalman指针


static model task;

float look_moto_yaw = 0;

/*
函数内容：云台任务
入口参数 无
返回值： 无
*/
	


void model_task(void *pvParameters)  
{ 
	debug_y_sk = 50;//45;//35;//14.8;//移动预测系数,越大预测越多
	debug_y_sb_sk = 59;//55;
	debug_y_sb_brig_sk = 90;//
	debug_p_sk = 20;//移动预测系数,越大预测越多
	debug_auto_err_y = 120;//角度过大关闭预测
	debug_auto_err_p = 150;
	debug_kf_delay = 80;//预测延时开启
	debug_kf_speed_yl = 0.2;//0.35;//速度过低关闭预测
	debug_kf_speed_yl_sb = 0.2;//0.2;//
	debug_kf_speed_yh = 5;//速度过高关闭预测
	debug_kf_speed_pl = 0.15;//pitch速度过低关闭预测
	debug_kf_y_angcon = 220;//125;//115;//135;//预测量限幅
	debug_kf_p_angcon = 45;//pitch预测量限幅
	
  //卡尔曼滤波器初始化
	/*PID角度误差卡尔曼,一阶*/
	KalmanCreate(&Gimbal_Pitch_Mech_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Pitch_Gyro_Error_Kalman, 1, 40);
	
	KalmanCreate(&Gimbal_Yaw_Mech_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Yaw_Gyro_Error_Kalman, 1, 40);
	
	  /*自瞄卡尔曼滤波,二阶*/
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
	
	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);	
	
	
	 
	  //yaw   整定完事
	 // pid_incr_init(&pid_6020_yaw,180,0.9,0,30000,5000);  //6020速度环
	 //pid_init(&pid_yaw_cirle,25,0,0,200,50,50,200);  //6020角度环


  pid_incr_init(&pid_6020_yaw,160,0.9,0,30000,5000);  //6020速度环
  pid_init(&pid_yaw_cirle,20,0,0,200,50,50,200);  //6020角度环
	  //pitch  
	 // pid_init(&pid_6020_pitch,235,5,0,30000,5000,10000,30000);  //6020速度环
	  //pid_init(&pid_pitch_cirle,20,0,0,200,50,100,200);  //6020角度环
	
  pid_init(&pid_6020_pitch,220,5,0,30000,5000,5000,30000);  //6020速度环
  pid_init(&pid_pitch_cirle,20,0,0,200,50,50,200);  //6020角度环
	  //得到各处指针地址，信息获取
		vis = get_vision_date();                              //视觉信息
	  moto_can = getmotosend();                             //can发送数据   
	  bmi = mpu_get_data();                                 //mpu部分数据
	 chassic = getchassic_flag();
  
	while(1)
			{
				task.time_flag++;
				if( task.time_flag >= ins_zero_flag + 2000 )  
				{  
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
				}
				vTaskDelay(1);	
				}
  }	
	
	

/*
函数内容：蜂鸣器鸣叫
入口参数 无
返回值： 无
*/
	
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



/*
函数内容：识别到后视觉自瞄
入口参数 无
返回值： 无
*/


void vis_get(void)
{
	 TIM_SetCompare3(TIM4,10000);
	 GIMBAL_AUTO_Mpde_Ctrl();
	
	 task.wait_y = bmi->pit;
	 task.wait_x = bmi->yaw;
	
	 task.sety  = task.sety  < -3 ? -3 : task.sety ;
	 task.sety  = task.sety  >  55 ?  55 : task.sety ;  //云台pitch限位	
	
		
	 /////////////////               yaw          /////////////

	 moto_can->yaw_cirle =	Calculate_Current_Value( &pid_yaw_cirle, task.setx , bmi->yaw );
	 moto_can->yaw  =	pid_incr_calc( &pid_6020_yaw,task.speed_x,moto_can->yaw_cirle );
		
	 ////////////////                pitch       ///////////    
	 moto_can->pitch_cirle = Calculate_Current_Value( &pid_pitch_cirle, task.sety , bmi->pit ); //角度环  
	 moto_can->pitch  =	-Calculate_Current_Value( &pid_6020_pitch, moto_can->pitch_cirle, task.speed_y ); //速度环
		
	 task.vis_time = 0;
}


/*
函数内容：未识别到目标巡航模式
入口参数 无
返回值： 无
*/

void vis_cruise(void)
{
	if( chassic->bee_flag == 0)
	   TIM_SetCompare3(TIM4,0);
		 task.vis_time++;
					 if( task.vis_time >= 800 )
					 {
						 //yaw轴巡航
					  moto_can->yaw  =	pid_incr_calc( &pid_6020_yaw,task.speed_x, -100 );
						//pitch轴巡航						
									 if( task.flag_pit == 0){
										if( bmi->pit  < 0 )
										{
											 task.flag_pit = 1; 
										}
										else
											  task.sety = task.sety - 0.15f;
									  }
									
							    else{
									 if( bmi->pit  >  50  )
									 {
										 task.flag_pit = 0;		
									 }										 
									 else
									     task.sety = task.sety + 0.15f; 
								   }
						
				 	  task.sety  = task.sety  < -3 ? -3 : task.sety ;
			      task.sety  = task.sety  >  55 ?  55 : task.sety ;  //云台pitch限位										 
	          moto_can->pitch_cirle = Calculate_Current_Value( &pid_pitch_cirle, task.sety , bmi->pit ); //角度环  
					  moto_can->pitch  =	-Calculate_Current_Value( &pid_6020_pitch, moto_can->pitch_cirle, task.speed_y ); //速度环							
						}	
					 else
					 {
					    moto_can->yaw_cirle =	Calculate_Current_Value( &pid_yaw_cirle, task.wait_x , bmi->yaw );
				      moto_can->yaw  =	pid_incr_calc( &pid_6020_yaw,task.speed_x,moto_can->yaw_cirle );
						 
					    moto_can->pitch_cirle = Calculate_Current_Value( &pid_pitch_cirle, task.wait_y , bmi->pit ); //角度环  
				      moto_can->pitch  =	-Calculate_Current_Value( &pid_6020_pitch, moto_can->pitch_cirle, task.speed_y ); //速度环
					 }		
	
				 					 
}


/*
函数内容：卡尔曼滤波
入口参数 无
返回值： 无
*/


void GIMBAL_AUTO_Mpde_Ctrl(void)
{
	static float yaw_angle_raw, pitch_angle_raw;//卡尔曼滤波角度测量值
	static float yaw_angle_ref;//记录目标角度
	static float pitch_angle_ref;//记录目标角度
	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓数据更新↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	if(vis->Vision_Get_New_Data == 1)
	{
		yaw_angle_ref   = bmi->yaw   + vis->set_yaw.angle   * error_yaw_k;
		pitch_angle_ref = bmi->pit   + vis->set_pitch.angle   * error_pitch_k;
		vis->Vision_Get_New_Data = 0; 	//标志位清除
		Vision_Time[NOW] = xTaskGetTickCount();
	}
	/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑数据更新↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓二阶卡尔曼计算↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	if(Vision_Time[NOW] != Vision_Time[LAST])//更新新数据到来的时间
	{ 
		vision_time_js = Vision_Time[NOW] - Vision_Time[LAST];//计算视觉延迟
		yaw_angle_raw  = yaw_angle_ref;//更新二阶卡尔曼滤波测量值
		pitch_angle_raw = pitch_angle_ref;
		Vision_Time[LAST] = Vision_Time[NOW];
	}
	if(vis->command == 1) //  识别到装甲板
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, Vision_Time[NOW], yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, Vision_Time[NOW], pitch_angle_raw);
				//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, Vision_Angle_Speed_Yaw);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, Vision_Angle_Speed_Pitch);
		/*gimbal_control.yaw_set = */
		yaw_kf_result_1 = yaw_kf_result[KF_ANGLE];
		pitch_kf_result_0 = pitch_kf_result[KF_ANGLE];  //pitch
			task.setx =  yaw_kf_result_1;
		  task.sety =  pitch_kf_result_0;
	}
	else
	{
		//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), bmi->yaw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), bmi->pit);
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, bmi->yaw, 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, bmi->pit, 0);
		/*gimbal_control.yaw_set = */
		yaw_kf_result_1 = yaw_kf_result[KF_ANGLE];
		pitch_kf_result_0 = pitch_kf_result[KF_ANGLE];
			task.setx = yaw_kf_result_1;
		  task.sety = pitch_kf_result_0;
		//debug_kf_angle_temp = 0;
	}
	if( vis->command == 1)
	{
			task.setx =  yaw_kf_result_1;
		  task.sety = pitch_kf_result_0;
	}
	
}




float debug_speed;

float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time);//计算速度

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}
	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//时间过长则认为速度不变
	}
	debug_speed = S->processed_speed;
	return S->processed_speed;//计算出的速度
}

model*gettasksend(void)
{
	return &task;
}


