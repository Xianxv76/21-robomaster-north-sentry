#ifndef		_RM_PID_H_
#define		_RM_PID_H_

#include "sys.h"


typedef struct PID
{
	float kp;
	float ki;
	float kd;
	
	float pout;
	float iout;
	float dout;
	
	float poutmax;
	float ioutmax;
	float doutmax;
	float outmax;
	
	float set;
	float real;
	float out;
	
	float err;							//����ƫ��ֵ
	float err_last;					//��һ��ƫ��ֵ
	float err_llast;				//���ϴ�ƫ��ֵ
	float integral;					//�ۼ�ƫ��ֵ
	void(*f_pid_init)(struct PID *pid, float kp, float ki, float kd, float poutmax, float ioutmax, float doutmax, float outmax);			//������ʼ��pid
	void(*f_pid_reset)(struct PID *pid);
}PID;

typedef struct
{
	 //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������
	
    float set;
    float fdb;	

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�	
	
}PID_INCR;


void pid_incr_init(PID_INCR *pid,float kp,float ki,float kd, float max_out, float max_iout);
float pid_incr_calc(PID_INCR *pid, float ref, float set);

void pid_init(PID *pid, float kp, float ki, float kd, float poutmax, float ioutmax, float doutmax, float outmax);


float Calculate_Current_Value(PID *pid, float set, float real);
float Calculate_Current_Value_For_Err(PID *pid, float err);



#endif
