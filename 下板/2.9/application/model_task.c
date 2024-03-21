#include "model_task.h"

#define NOW 0
#define LAST 1
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2

 static PID_INCR pid_6020_yaw;   //6020�ٶȻ�
 static PID      pid_yaw_cirle;   //6020�ǶȻ�

 static PID  pid_6020_pitch;   //6020�ٶȻ�
 static PID       pid_pitch_cirle;   //6020�ǶȻ� 

 static moto_send*moto_can;     //can ����
 static vision*vis;
 
 static mpu*bmi;
 
 static flag*chassic;
int vision_time_js;

uint32_t Vision_Time[2];// NOW/LAST

float error_yaw_k   = 1;
float error_pitch_k = 1;

float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//�������˲��ٶȲ���ֵ
float *yaw_kf_result, *pitch_kf_result;//���׿������˲����,0�Ƕ� 1�ٶ�
float pitch_kf_result_0, yaw_kf_result_1;

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;
speed_calc_data_t Vision_Yaw_speed_Struct;
speed_calc_data_t Vision_Pitch_speed_Struct;

uint32_t Vision_Time[2];// NOW/LAST

kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.001/*0.002*/, 0, 1},//����ʱ����
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {500, 0, 0, 1000}//500 1000
};//��ʼ��yaw�Ĳ���kalman����

kalman_filter_init_t pitch_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.001/*0.002*/, 0, 1},//����ʱ����
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}
};//��ʼ��pitch�Ĳ���kalman����


float debug_y_sk;// = 38;//35;//30;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
float debug_y_sb_sk;//�ڱ�Ԥ��ϵ��
float debug_y_sb_brig_sk;//��ͷ�ڱ�
float debug_p_sk;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
float debug_auto_err_y;// = 10;//15;//10;//15;//yaw�Ƕȹ���ر�Ԥ��
float debug_auto_err_p;//pitch�Ƕȹ���ر�Ԥ��
float debug_kf_delay;// = 150;//100;//200;//120;//150;//Ԥ����ʱ����
float debug_kf_speed_yl;//yaw�ٶȹ��͹ر�Ԥ��
float debug_kf_speed_yl_sb;//̧ͷ���ڱ�ʱ��С��Ϳɿ�Ԥ����
float debug_kf_speed_yh;//yaw�ٶȹ��߹ر�Ԥ��
float debug_kf_speed_pl;//pitch�ٶȹ��͹ر�Ԥ��
float debug_kf_y_angcon;// = 130;//125;//115;//135;//yawԤ�����޷�
float debug_kf_p_angcon;//pitchԤ�����޷�
 

/*һ�׿�����*/
//��̨�Ƕ�������
extKalman_t Gimbal_Pitch_Mech_Error_Kalman;//����һ��kalmanָ��
extKalman_t Gimbal_Pitch_Gyro_Error_Kalman;//����һ��kalmanָ��
extKalman_t Gimbal_Yaw_Mech_Error_Kalman;//����һ��kalmanָ��
extKalman_t Gimbal_Yaw_Gyro_Error_Kalman;//����һ��kalmanָ��


static model task;

float look_moto_yaw = 0;

/*
�������ݣ���̨����
��ڲ��� ��
����ֵ�� ��
*/
	


void model_task(void *pvParameters)  
{ 
	debug_y_sk = 50;//45;//35;//14.8;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
	debug_y_sb_sk = 59;//55;
	debug_y_sb_brig_sk = 90;//
	debug_p_sk = 20;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
	debug_auto_err_y = 120;//�Ƕȹ���ر�Ԥ��
	debug_auto_err_p = 150;
	debug_kf_delay = 80;//Ԥ����ʱ����
	debug_kf_speed_yl = 0.2;//0.35;//�ٶȹ��͹ر�Ԥ��
	debug_kf_speed_yl_sb = 0.2;//0.2;//
	debug_kf_speed_yh = 5;//�ٶȹ��߹ر�Ԥ��
	debug_kf_speed_pl = 0.15;//pitch�ٶȹ��͹ر�Ԥ��
	debug_kf_y_angcon = 220;//125;//115;//135;//Ԥ�����޷�
	debug_kf_p_angcon = 45;//pitchԤ�����޷�
	
  //�������˲�����ʼ��
	/*PID�Ƕ�������,һ��*/
	KalmanCreate(&Gimbal_Pitch_Mech_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Pitch_Gyro_Error_Kalman, 1, 40);
	
	KalmanCreate(&Gimbal_Yaw_Mech_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Yaw_Gyro_Error_Kalman, 1, 40);
	
	  /*���鿨�����˲�,����*/
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
	
	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);	
	
	
	 
	  //yaw   ��������
	 // pid_incr_init(&pid_6020_yaw,180,0.9,0,30000,5000);  //6020�ٶȻ�
	 //pid_init(&pid_yaw_cirle,25,0,0,200,50,50,200);  //6020�ǶȻ�


  pid_incr_init(&pid_6020_yaw,160,0.9,0,30000,5000);  //6020�ٶȻ�
  pid_init(&pid_yaw_cirle,20,0,0,200,50,50,200);  //6020�ǶȻ�
	  //pitch  
	 // pid_init(&pid_6020_pitch,235,5,0,30000,5000,10000,30000);  //6020�ٶȻ�
	  //pid_init(&pid_pitch_cirle,20,0,0,200,50,100,200);  //6020�ǶȻ�
	
  pid_init(&pid_6020_pitch,220,5,0,30000,5000,5000,30000);  //6020�ٶȻ�
  pid_init(&pid_pitch_cirle,20,0,0,200,50,50,200);  //6020�ǶȻ�
	  //�õ�����ָ���ַ����Ϣ��ȡ
		vis = get_vision_date();                              //�Ӿ���Ϣ
	  moto_can = getmotosend();                             //can��������   
	  bmi = mpu_get_data();                                 //mpu��������
	 chassic = getchassic_flag();
  
	while(1)
			{
				task.time_flag++;
				if( task.time_flag >= ins_zero_flag + 2000 )  
				{  
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
				}
				vTaskDelay(1);	
				}
  }	
	
	

/*
�������ݣ�����������
��ڲ��� ��
����ֵ�� ��
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
�������ݣ�ʶ�𵽺��Ӿ�����
��ڲ��� ��
����ֵ�� ��
*/


void vis_get(void)
{
	 TIM_SetCompare3(TIM4,10000);
	 GIMBAL_AUTO_Mpde_Ctrl();
	
	 task.wait_y = bmi->pit;
	 task.wait_x = bmi->yaw;
	
	 task.sety  = task.sety  < -3 ? -3 : task.sety ;
	 task.sety  = task.sety  >  55 ?  55 : task.sety ;  //��̨pitch��λ	
	
		
	 /////////////////               yaw          /////////////

	 moto_can->yaw_cirle =	Calculate_Current_Value( &pid_yaw_cirle, task.setx , bmi->yaw );
	 moto_can->yaw  =	pid_incr_calc( &pid_6020_yaw,task.speed_x,moto_can->yaw_cirle );
		
	 ////////////////                pitch       ///////////    
	 moto_can->pitch_cirle = Calculate_Current_Value( &pid_pitch_cirle, task.sety , bmi->pit ); //�ǶȻ�  
	 moto_can->pitch  =	-Calculate_Current_Value( &pid_6020_pitch, moto_can->pitch_cirle, task.speed_y ); //�ٶȻ�
		
	 task.vis_time = 0;
}


/*
�������ݣ�δʶ��Ŀ��Ѳ��ģʽ
��ڲ��� ��
����ֵ�� ��
*/

void vis_cruise(void)
{
	if( chassic->bee_flag == 0)
	   TIM_SetCompare3(TIM4,0);
		 task.vis_time++;
					 if( task.vis_time >= 800 )
					 {
						 //yaw��Ѳ��
					  moto_can->yaw  =	pid_incr_calc( &pid_6020_yaw,task.speed_x, -100 );
						//pitch��Ѳ��						
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
			      task.sety  = task.sety  >  55 ?  55 : task.sety ;  //��̨pitch��λ										 
	          moto_can->pitch_cirle = Calculate_Current_Value( &pid_pitch_cirle, task.sety , bmi->pit ); //�ǶȻ�  
					  moto_can->pitch  =	-Calculate_Current_Value( &pid_6020_pitch, moto_can->pitch_cirle, task.speed_y ); //�ٶȻ�							
						}	
					 else
					 {
					    moto_can->yaw_cirle =	Calculate_Current_Value( &pid_yaw_cirle, task.wait_x , bmi->yaw );
				      moto_can->yaw  =	pid_incr_calc( &pid_6020_yaw,task.speed_x,moto_can->yaw_cirle );
						 
					    moto_can->pitch_cirle = Calculate_Current_Value( &pid_pitch_cirle, task.wait_y , bmi->pit ); //�ǶȻ�  
				      moto_can->pitch  =	-Calculate_Current_Value( &pid_6020_pitch, moto_can->pitch_cirle, task.speed_y ); //�ٶȻ�
					 }		
	
				 					 
}


/*
�������ݣ��������˲�
��ڲ��� ��
����ֵ�� ��
*/


void GIMBAL_AUTO_Mpde_Ctrl(void)
{
	static float yaw_angle_raw, pitch_angle_raw;//�������˲��ǶȲ���ֵ
	static float yaw_angle_ref;//��¼Ŀ��Ƕ�
	static float pitch_angle_ref;//��¼Ŀ��Ƕ�
	
	/*�����������������������������������������������ݸ��¡�������������������������������������������������������������*/
	if(vis->Vision_Get_New_Data == 1)
	{
		yaw_angle_ref   = bmi->yaw   + vis->set_yaw.angle   * error_yaw_k;
		pitch_angle_ref = bmi->pit   + vis->set_pitch.angle   * error_pitch_k;
		vis->Vision_Get_New_Data = 0; 	//��־λ���
		Vision_Time[NOW] = xTaskGetTickCount();
	}
	/*���������������������������������������������������ݸ��¡���������������������������������������������������������*/
	
	/*�����������������������������������������������׿����������������������������������������������������������������������*/
	if(Vision_Time[NOW] != Vision_Time[LAST])//���������ݵ�����ʱ��
	{ 
		vision_time_js = Vision_Time[NOW] - Vision_Time[LAST];//�����Ӿ��ӳ�
		yaw_angle_raw  = yaw_angle_ref;//���¶��׿������˲�����ֵ
		pitch_angle_raw = pitch_angle_ref;
		Vision_Time[LAST] = Vision_Time[NOW];
	}
	if(vis->command == 1) //  ʶ��װ�װ�
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, Vision_Time[NOW], yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, Vision_Time[NOW], pitch_angle_raw);
				//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
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
		//		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
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
		S->speed = (position - S->last_position) / (time - S->last_time);//�����ٶ�

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}
	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//ʱ���������Ϊ�ٶȲ���
	}
	debug_speed = S->processed_speed;
	return S->processed_speed;//��������ٶ�
}

model*gettasksend(void)
{
	return &task;
}


