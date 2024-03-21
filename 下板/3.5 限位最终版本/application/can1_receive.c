#include "can1_receive.h"

/*
�ļ����� ��can1���ݽ���
��;��     ���ܵ��̵��3510 ������2006��� ��̨pitch��yaw����ͨ��can1���͵���Ϣ
ʱ�䣺     2020 12 1
��д�ˣ�   gu
*/

//����������λ��
static encoder0*encoder_cm ;
 
static judgement judge;

 /*
�����Ϣ
*/
//0 3510        1 2006        2 6020pitch       3 6020yaw 
 motor_date_ty motor_date[4] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	                               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	                               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

/*
�������ݣ�ͨ��id����can��Ϣ

��ڲ�����  CanRxMsg *message  can1������

����ֵ�� ��

*/

void CanReceiveMsgProcess(CanRxMsg *message)
{  
	//ָ���ַ
	encoder_cm = get_encoder0_data();
	switch(message->StdId)
	{
		case CAN_ID_CM5:     encoder_cm->now_echo = ( message->Data[0]<<8 )| message->Data[1];    //��������Ϣ
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
�������ݣ��õ������Ϣ  

��ڲ���1��   Encoder �ṹ�壨��ŵ����Ϣ��
    ����2     ���data���� 

����ֵ�� ��

*/

void getEncoderData( motor_date_ty *v, CanRxMsg * msg)    //����������
{
	int i=0;
	int32_t temp_sum = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)| msg->Data[1];
	if( v->first_flag == 0 )
	{v->ecd_bias = v->raw_value;   //��һ�α�������ֵ
	  v->first_flag =1 ;}
	else
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�
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
	//����õ������ı��������ֵ
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	//����õ��Ƕ�ֵ����Χ���������
	
	
	v->ecd_ang[1] =	(float)(v->raw_value )*360/8192 + v->round_cnt * 360;
	
	v->ecd_ang[0] = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	
	v->ecd_angle = v->ecd_angle + ( v->ecd_ang[0] - v->ecd_ang[1] );
	
	//v->ecd_ang[1] =	v->ecd_ang[0] ;
	
	
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate * 360 * 1000 /8192.f  ;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//�����ٶ�ƽ��ֵ
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->ecd_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		
}



/*
�������ݣ��ⲿ�����õ�3508��Ϣ  
��ڲ��� ָ��3508�������Ϣ��ָ��
����ֵ�� 
*/
 motor_date_ty * getstandmotorpoint(void)   //3508��Ϣ
{
	return &motor_date[0];
}

/*
�������ݣ��ⲿ�����õ�2006��Ϣ  
��ڲ��� ָ��2006�������Ϣ��ָ��
����ֵ�� 
*/

 motor_date_ty * getfiremotorpoint(void)    //2006��Ϣ
{
	return &motor_date[1];
}


/*
�������ݣ��ⲿ�����õ�6020 pitch��Ϣ  
��ڲ��� ָ��6020pitch�������Ϣ��ָ��
����ֵ�� 
*/

 motor_date_ty * get6020pitchmotorpoint(void)    //6020pitch��Ϣ
{
	return &motor_date[2];
}


/*
�������ݣ��ⲿ�����õ�6020 yaw��Ϣ  
��ڲ��� ָ��6020yaw�������Ϣ��ָ��
����ֵ�� 
*/

 motor_date_ty * get6020yawmotorpoint(void)    //6020yaw��Ϣ
{
	return &motor_date[3];
}


judgement*getjudugementdate(void)
{ 
	 return &judge;
}

