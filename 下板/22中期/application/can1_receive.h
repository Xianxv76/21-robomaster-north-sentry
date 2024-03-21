#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include<stdlib.h>
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "encoder_task.h"


#define CAN_ID_CM1 0x201
#define CAN_ID_CM2 0x202
#define CAN_ID_CM3 0x206
#define CAN_ID_CM4 0x205
#define CAN_ID_CM5 0x1FA

#define RATE_BUF_SIZE 6

//�����Ϣ

typedef struct{
	int32_t raw_value;   					 //���������������ԭʼֵ
	int32_t last_raw_value;					 //��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;							 //���α�����֮��Ĳ�ֵ
	int32_t temp_count;                      //������
	uint8_t buf_count;						 //�˲�����buf��
	int32_t ecd_bias;						 //��ʼ������ֵ	
	int32_t ecd_raw_rate;					 //ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	     //buf��for filter
	int32_t round_cnt;						 //Ȧ��
	int32_t ecd_rate;					 //�ٶ�
	int32_t first_flag;        //�����һ�α�����λ�õı�־λ
	float   ecd_angle;						 //�Ƕ�
	float   ecd_ang[2];               //�Ƕ��ۼ�
		
}motor_date_ty;

typedef struct{
	
	int shoot_heat;
	int admier_id;
	int robot_id;
}judgement;



void CanReceiveMsgProcess(CanRxMsg *message);

void getEncoderData( motor_date_ty *v, CanRxMsg * msg);



//�ⲿ��ͨ����

 motor_date_ty * getstandmotorpoint(void);
	
 motor_date_ty * getfiremotorpoint(void);

 motor_date_ty * get6020pitchmotorpoint(void);

 motor_date_ty * get6020yawmotorpoint(void);

 judgement*getjudugementdate(void);
	

#endif
