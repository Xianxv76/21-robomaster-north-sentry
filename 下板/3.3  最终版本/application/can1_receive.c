#include "can1_receive.h"

/*
文件内容 ：can1数据接受
用途：     接受底盘电机3510 拨弹盘2006电机 云台pitch和yaw轴电机通过can1发送的信息
时间：     2020 12 1
编写人：   gu
*/

//编码器数据位置
static encoder0*encoder_cm ;
 
static judgement judge;

 /*
电机信息
*/
//0 3510        1 2006        2 6020pitch       3 6020yaw 
 motor_date_ty motor_date[4] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	                               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	                               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

/*
函数内容：通过id分离can信息

入口参数：  CanRxMsg *message  can1的数据

返回值： 无

*/

void CanReceiveMsgProcess(CanRxMsg *message)
{  
	//指针地址
	encoder_cm = get_encoder0_data();
	switch(message->StdId)
	{
		case CAN_ID_CM5:     encoder_cm->now_echo = ( message->Data[0]<<8 )| message->Data[1];    //编码器信息
			                   judge.admier_id = ( message->Data[2]<<8 )| message->Data[3]; 
		                     judge.shoot_heat = ( message->Data[4]<<8 )| message->Data[5];        
		                     judge.robot_id = ( message->Data[6]<<8 )| message->Data[7]; 
		                     break;
		
    case CAN_ID_CM1:     getEncoderData( &motor_date[0],message );       break;  
					        
		case CAN_ID_CM2:     getEncoderData( &motor_date[1],message ); 	     break;  //2006

		case CAN_ID_CM3:  	 getEncoderData( &motor_date[2],message  );   	 break;  //6020 pitch

		case CAN_ID_CM4:	   getEncoderData( &motor_date[3],message  );   	 break;  //6020 yaw
	}
}



/*
函数内容：得到电机信息  

入口参数1：   Encoder 结构体（存放电机信息）
    参数2     电机data数组 

返回值： 无

*/

void getEncoderData( motor_date_ty *v, CanRxMsg * msg)    //编码器解算
{
	int i=0;
	int32_t temp_sum = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)| msg->Data[1];
	if( v->first_flag == 0 )
	{v->ecd_bias = v->raw_value;   //第一次编码器的值
	  v->first_flag =1 ;}
	else
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff - 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	//计算得到角度值，范围正负无穷大
	
	
	v->ecd_ang[1] =	(float)(v->raw_value )*360/8192 + v->round_cnt * 360;
	
	v->ecd_ang[0] = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	
	v->ecd_angle = v->ecd_angle + ( v->ecd_ang[0] - v->ecd_ang[1] );
	
	//v->ecd_ang[1] =	v->ecd_ang[0] ;
	
	
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate * 360 * 1000 /8192.f  ;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->ecd_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		
}



/*
函数内容：外部函数得到3508信息  
入口参数 指向3508电机的信息的指针
返回值： 
*/
 motor_date_ty * getstandmotorpoint(void)   //3508信息
{
	return &motor_date[0];
}

/*
函数内容：外部函数得到2006信息  
入口参数 指向2006电机的信息的指针
返回值： 
*/

 motor_date_ty * getfiremotorpoint(void)    //2006信息
{
	return &motor_date[1];
}


/*
函数内容：外部函数得到6020 pitch信息  
入口参数 指向6020pitch电机的信息的指针
返回值： 
*/

 motor_date_ty * get6020pitchmotorpoint(void)    //6020pitch信息
{
	return &motor_date[2];
}


/*
函数内容：外部函数得到6020 yaw信息  
入口参数 指向6020yaw电机的信息的指针
返回值： 
*/

 motor_date_ty * get6020yawmotorpoint(void)    //6020yaw信息
{
	return &motor_date[3];
}


judgement*getjudugementdate(void)
{ 
	 return &judge;
}

