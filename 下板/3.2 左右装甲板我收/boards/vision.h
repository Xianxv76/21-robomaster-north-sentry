#ifndef _VISION_H
#define _VISION_H

#include<stdlib.h>
#include "sys.h"
#include "can1_receive.h"
#include "ins_task.h"



//16λ������
union int16
{
	unsigned char data[2];
	int16_t angle;
};    

//32λ������
union float32
{
	unsigned char data[4];
	float  angle;
};   

  //�Ӿ����սṹ��
typedef struct
{
	char mode;              //0���飬1���������أ�2С��������  //�ڱ�ֻ��0
  float  set_yaw; 	//yaw�Ƕ�
	union float32  set_yaw_data;
	union float32  set_left_yaw;
	union float32  set_right_yaw;
	union float32  set_pitch;         //pitch�Ƕ�
	//union float32 distance;         //Ŀ�����
	char command;           //�Ƿ��ҵ�Ŀ�� bool��
	int Vision_Get_New_Data; //�������˲��������ݱ�־λ
}vision;
  

void vision_init(void);
vision*get_vision_date(void);


#endif
