#ifndef RC_H
#define RC_H

#include "sys.h"

typedef struct
{
	int16_t R_x;  //����x		��1684 ��364 ��1024  1684-1024 = 1024 - 364 = 660
	int16_t R_y;	 //����y    ��1684 ��364 ��1024
	int16_t L_x;
	int16_t L_y;
	uint8_t sl;			//������  ��1 ��3 ��2
	uint8_t sr;
	
	int16_t mouse_x;	//���
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_l;	//��갴��
	uint8_t mouse_r;
	uint16_t key;		//����
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

