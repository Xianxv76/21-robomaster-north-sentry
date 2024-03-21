#ifndef _VISION_H
#define _VISION_H

#include "sys.h"
#include "can1_receive.h"
#include "ins_task.h"



//16位共用体
union int16
{
	unsigned char data[2];
	int16_t angle;
};    

//32位共用体
union float32
{
	unsigned char data[4];
	float  angle;
};   

  //视觉接收结构体
typedef struct
{
	char mode;              //0自瞄，1大能量机关，2小能量机关  //哨兵只用0
	union float32  set_yaw;           //yaw角度
	u32 last_yaw_angle;
	union float32  set_pitch;         //pitch角度
	u32 last_pitch_angle;
	union float32 distance;         //目标距离
	char command;           //是否找到目标 bool型
	float yaw_offset;
	float pitch_offset;
}vision;
  

void vision_init(void);
vision*get_vision_date(void);


#endif
