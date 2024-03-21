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
	
	float err;							//定义偏差值
	float err_last;					//上一次偏差值
	float err_llast;				//上上次偏差值
	float integral;					//累计偏差值
	void(*f_pid_init)(struct PID *pid, float kp, float ki, float kd, float poutmax, float ioutmax, float doutmax, float outmax);			//用来初始化pid
	void(*f_pid_reset)(struct PID *pid);
}PID;

typedef struct
{
	 //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出
	
    float set;
    float fdb;	

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次	
	
}PID_INCR;


void pid_incr_init(PID_INCR *pid,float kp,float ki,float kd, float max_out, float max_iout);
float pid_incr_calc(PID_INCR *pid, float ref, float set);

void pid_init(PID *pid, float kp, float ki, float kd, float poutmax, float ioutmax, float doutmax, float outmax);


float Calculate_Current_Value(PID *pid, float set, float real);
float Calculate_Current_Value_For_Err(PID *pid, float err);



#endif
