#ifndef _VISION_H
#define _VISION_H

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
	union float32  set_yaw;           //yaw�Ƕ�
	u32 last_yaw_angle;
	union float32  set_pitch;         //pitch�Ƕ�
	u32 last_pitch_angle;
	union float32 distance;         //Ŀ�����
	char command;           //�Ƿ��ҵ�Ŀ�� bool��
	float yaw_offset;
	float pitch_offset;
}vision;
  

void vision_init(void);
vision*get_vision_date(void);


#endif
