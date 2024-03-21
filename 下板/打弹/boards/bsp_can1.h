#ifndef _BSP_CAN1_H_
#define _BSP_CAN1_H_

#include "sys.h"
#include "can1_receive.h"


void CAN_Configure(void);  
void Set_ChassisMotor_Current(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void Set_CloudMotor_Current(int16_t cm1_iq, int16_t cm2_iq);


#endif
