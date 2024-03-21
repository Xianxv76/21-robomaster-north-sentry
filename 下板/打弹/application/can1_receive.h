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

//电机信息

typedef struct{
	int32_t raw_value;   					 //编码器不经处理的原始值
	int32_t last_raw_value;					 //上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;							 //两次编码器之间的差值
	int32_t temp_count;                      //计数用
	uint8_t buf_count;						 //滤波更新buf用
	int32_t ecd_bias;						 //初始编码器值	
	int32_t ecd_raw_rate;					 //通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	     //buf，for filter
	int32_t round_cnt;						 //圈数
	int32_t ecd_rate;					 //速度
	int32_t first_flag;        //保存第一次编码器位置的标志位
	float   ecd_angle;						 //角度
	float   ecd_ang[2];               //角度累加
		
}motor_date_ty;

typedef struct{
	
	int shoot_heat;
	int admier_id;
	int robot_id;
}judgement;



void CanReceiveMsgProcess(CanRxMsg *message);

void getEncoderData( motor_date_ty *v, CanRxMsg * msg);



//外部沟通函数

 motor_date_ty * getstandmotorpoint(void);
	
 motor_date_ty * getfiremotorpoint(void);

 motor_date_ty * get6020pitchmotorpoint(void);

 motor_date_ty * get6020yawmotorpoint(void);

 judgement*getjudugementdate(void);
	

#endif
