#ifndef RC_H
#define RC_H

#include "sys.h"

typedef struct
{
	int16_t R_x;  //右手x		右1684 左364 中1024  1684-1024 = 1024 - 364 = 660
	int16_t R_y;	 //右手y    上1684 下364 中1024
	int16_t L_x;
	int16_t L_y;
	uint8_t sl;			//拨动左  上1 中3 下2
	uint8_t sr;
	
	int16_t mouse_x;	//鼠标
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_l;	//鼠标按键
	uint8_t mouse_r;
	uint16_t key;		//键盘
	u8 W;
	u8 S;
	u8 A;
	u8 D;
	u8 Q;
	u8 E;
	u8 Shift;
	u8 Ctrl;
	
	uint64_t cnts;
	uint64_t off_line_times;
	uint64_t last_cnts;
	uint64_t now_cnts;
	uint64_t off_line_flag;
}RC;

void rc_init(void);
RC*get_rc_data(void);

#endif 

