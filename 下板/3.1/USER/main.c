#include "main.h"


int a= 0;

int main(void)
{ 
	int i =0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //����ϵͳ�ж����ȼ�����4
	delay_init(168);		     //��ʼ����ʱ����
	CAN_Configure();         //can1ͨѶ
//  KEY_Init();            //����
//	TIM4_PWM_Init();         //������
//	LED_Init();              //led��
//	vision_init();         //�Ӿ����մ���
//	TIM1_PWM_Init(2000,168); //Ħ���ֵ����ʼ��pwm
	rc_init();               //ң��������
	while(1)
	{}
		
//	bim088_spi_init();       //bmi088��spi�ĳ�ʼ��
//	
//	while(BMI088_init())     //�����ǳ�ʼ�� 
//    { ; }
//	 a = 1 ;                 //�����ǳ�ʼ�����
	
//	delay_ms(500);

//		for(i = 0; i<8;i++)    //�����ֳ�ʼ�� 
//	{
//		TIM_SetCompare1(TIM1,1100);
//		TIM_SetCompare2(TIM1,1100);
//		delay_ms(500);
//	}
	
//	start();

	}
	

