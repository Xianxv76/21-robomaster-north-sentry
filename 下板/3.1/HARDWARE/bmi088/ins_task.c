#include "ins_task.h"


 mpu bmi;

//float mpu_zero_kp = 0.000204;  //��Ưϵ��      //0.0115f   100 �ε�ֵ

//float low_pass_kp = 0.08f;



/*             ������        
�޴������	
*/


// 5����Ư

void bmi_get(void)
{
	int i;
	if(bmi.zero_flag<6000)
	  bmi.zero_flag++;
	else
		bmi.zero_flag = 6000;
	
	  //�õ����ٶ� ��λ��ת��Ϊ�Ƕ�
	 BMI088_read(bmi.gyro_buf, bmi.accel_buf, &bmi.temp_buf);   
		//��ͨ�˲�	
	 bmi_lowpass(bmi.gyro_buf,bmi.gyro_pass,bmi.accel_buf,bmi.accel_pass);    
	
	//ȡ5000��   ������Ư
	if(bmi.zero_flag>1000 && bmi.zero_flag<6000)                                    
	 bmi_zero_val(bmi.gyro_buf,bmi.gyro_zero);  
 
	if(bmi.zero_flag == 6000)
	{
		//���ٶȼ�ȥ��Ư
		for(i = 0; i<=2; i++)
		bmi.gyro[i] = bmi.gyro_buf[i] - bmi.gyro_zero[i];
    //��Ԫ������
		//bmi_ins(accel_pass[0],accel_pass[1],accel_pass[2],gyro[0],gyro[1],gyro[2]);
	
		  bmi_ins(bmi.accel_pass[0],bmi.accel_pass[1],bmi.accel_pass[2],bmi.gyro[0],bmi.gyro[1],bmi.gyro[2]);  //���ٶ��˲������ٶȲ��˲� ���ٶȴ������Ư���ֵ
	}
}


/*             ��ͨ�˲�         
�������
1. ���ٶ�ԭʼֵ
2. ���ٶ��˲�֮���ֵ
3. ���ٶ�ԭʼֵ
4. ���ٶ��˲�֮���ֵ
*/

void bmi_lowpass(float *gyro_buf, float *gyro_pass, float*accel_buf, float*accel_pass)
{
	int i;
	for(i = 0; i<=2 ;i++)
	{
		gyro_pass[i] = ( gyro_buf[i] * bmi.low_pass_kp ) + ( 1 - bmi.low_pass_kp) * gyro_pass[i];
		accel_pass[i] = ( accel_buf[i] * bmi.low_pass_kp ) + ( 1 - bmi.low_pass_kp) * accel_pass[i];
	}
}

/*             ��Ư�ļ���          
�������
1.���ٶ�ԭʼֵ
2.���ٶ���Ưֵ

*/

void bmi_zero_val(float *gyro,float *gyro_zero)            
{
	int i;
	for(i = 0; i<=2; i++)
	gyro_zero[i] = gyro_zero[i] + gyro[i] * bmi.mpu_zero_kp;
}


/*     ��Ԫ������
�������
1.���ٶ�ֵ
2
4.���ٶ�ֵ
5
*/

void bmi_ins(float ax, float ay, float az, float gx, float gy, float gz)
{ 
	static float q0=1,q1=0,q2=0,q3=0,
		           exint=0,eyint=0,ezint=0,
	             mpu_ki=0.0f,mpu_kp=3.0f,
	             vx=0,vy=0,vz=0,
	             ex=0,ey=0,ez=0,
	             norm=0,last = 0, now = 0,
	             halfT = 0;
	 last=now;
   now=xTaskGetTickCount();
	
   halfT = (now-last) * 0.0005f;

    if(ax*ay*az==0)
        return;
		
  norm = sqrt(ax*ax + ay*ay + az*az);       //acc���ݹ�һ��
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;
 
  // estimated direction of gravity and flux (v and w)              �����������������/��Ǩ
  vx = 2*(q1*q3 - q0*q2);                                             //��Ԫ����xyz�ı�ʾ
  vy = 2*(q0*q1 + q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3 ;
 
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                                             //�������������õ���־������
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;
 
  exint = exint + ex * mpu_ki;                                //�������л���
  eyint = eyint + ey * mpu_ki;
  ezint = ezint + ez * mpu_ki;
 
  // adjusted gyroscope measurements
  gx = gx + mpu_kp*ex + exint;                                              //�����PI�󲹳��������ǣ����������Ư��
  gy = gy + mpu_kp*ey + eyint;
  gz = gz + mpu_kp*ez + ezint; 
                                         //�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�
//  ex1=Kp*ex + exInt;
//  ey1=Kp*ey + eyInt;
//  ey1=Kp*ez + ezInt;
  // integrate quaternion rate and normalise   //��Ԫ�ص�΢�ַ���
                          												 
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;  //1/2000.f;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;   //1/2000.f;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;   //1/2000.f;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;   //1/2000.f;
 
  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
 
  bmi.yaw1 = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3 ;// yaw
	
	if( bmi.first_flag == 1 )
		{
			if( bmi.yaw1 - bmi.yaw_last >= 370 || bmi.yaw1 - bmi.yaw_last <= -370 )
				bmi.yaw_last = bmi.yaw1;                             //ȥ������ȶ���
			
			if(bmi.yaw1 - bmi.yaw_last <= -150)
			 bmi.count++;
			else if(bmi.yaw1 - bmi.yaw_last >= 150)
			 bmi.count--;
			bmi.yaw = bmi.yaw1 + bmi.count * 360.f - bmi.yaw_first;
			bmi.yaw_last = bmi.yaw1;                
		}  
	else
		{
			bmi.first_time++;
			if( bmi.first_time >= 300 )
			 bmi.first_flag = 1;
			 bmi.yaw_last = bmi.yaw1; 
			 bmi.yaw_first = bmi.yaw1;          //�õ������ǵ�������ֵ
		}
	
  bmi.pit = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  bmi.rol = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
}

mpu*mpu_get_data(void){
	
	return &bmi;
	
}

 //  �����ǽ�������
void ins_task(void *pvParameters)
{
	
    while(1)
    {
       bmi_get();
			
     vTaskDelay(1);
    }
}   


