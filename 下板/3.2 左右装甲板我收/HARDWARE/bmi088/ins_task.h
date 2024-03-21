#ifndef _INS_TASK_H_
#define _INS_TASK_H_
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "BMI088driver.h"
#include "BMI088_INT.h"
#include "math.h"

#define rad 57.296f

#define ins_zero_flag 5000

typedef struct{
	int zero_flag ;
	int first_flag;
	int first_time;	
	float gyro_buf[3], accel_buf[3], temp_buf;
	float mpu_zero_kp;
	float gyro_pass[3],
	      accel_pass[3],
	      gyro_zero[3];
	float gyro[3];
  float yaw, pit, rol, yaw1, yaw_last, count, yaw_first;
	float low_pass_kp;
	
}mpu;



void bmi_zero_val(float *gyro,float *gyro_zero); 
void bmi_get(void);
void bmi_lowpass(float *gyro_buf, float *gyro_pass, float*accel_buf, float*accel_pass);
void bmi_ins(float ax, float ay, float az, float gx, float gy, float gz);	
mpu*mpu_get_data(void);


#endif
