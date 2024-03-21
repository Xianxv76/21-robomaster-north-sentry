#ifndef _BSP_CAN1_H_
#define _BSP_CAN1_H_

#include "sys.h"

void CAN_Configure(void);  
void can1_send_encoder(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq );

#endif
