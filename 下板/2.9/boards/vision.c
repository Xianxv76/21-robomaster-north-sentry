#include "vision.h"
#include "string.h"

#define RX_VISION_SIZE 18		//DMA���ջ�������С

 static u8 RXVISION[RX_VISION_SIZE];		//DMA���ջ�����
  vision vis;
 
 static mpu*bmi;
 
 
void vision_init(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;		//dma�õ��ж�
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);;//ʹ��USART1ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); //DMA2ʱ��ʹ�� ---********************
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;         
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;       
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;       
    GPIO_Init(GPIOG, &GPIO_InitStructure);  /* TXIO */  

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;                
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;       
    GPIO_Init(GPIOG, &GPIO_InitStructure);  /* RXIO */

  //USART1 ��ʼ������
	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate = 115200;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//��У��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_Init(USART6, &USART_InitStructure); //��ʼ������1
	
	USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���1
	
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);  //ʹ�ܴ���1��DMA����     
	
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;//DMA1 ������5 �ж�ͨ��*******
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
	
  /* ���� DMA Stream */
	DMA_DeInit(DMA2_Stream1); 	
	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);//�ȴ�DMA������
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART6->DR);//DMA�����ַ****************************
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)RXVISION;//DMA �洢��0��ַ �ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�洢��ģʽ*******************************
	DMA_InitStructure.DMA_BufferSize = RX_VISION_SIZE;//���ݴ����� 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//�е����ȼ�**********
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//***************
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���*****
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���

	DMA_Init(DMA2_Stream1, &DMA_InitStructure);//��ʼ��DMA2 Stream5	
	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA2_Stream1,ENABLE);
}

float look_set_yaw = 0;
float look_set_pitch = 0;


void DMA2_Stream1_IRQHandler(void)
{
	 bmi = mpu_get_data();
	if(DMA_GetFlagStatus(DMA2_Stream1,DMA_FLAG_TCIF1)!=RESET)
	{
		DMA_Cmd(DMA2_Stream1, DISABLE); //�ر�DMA,��ֹ�������������		
		DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1);//���DMA2_Steam7������ɱ�־
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);		

		 if( RXVISION[0] == 0xA5 && RXVISION[15] == 0xEE )
	 {
		
		 vis.mode = RXVISION[1];     //ģʽ
		 ///////////yaw��///////////
		 vis.set_yaw.data[0] = RXVISION[2] ;  
		 vis.set_yaw.data[1] = RXVISION[3] ;  
     vis.set_yaw.data[2] = RXVISION[4] ;  
		 vis.set_yaw.data[3] = RXVISION[5] ; 
		 if( vis.set_yaw.angle == 45 )
			 vis.set_yaw.angle = 0;
		 vis.set_yaw.angle =  -vis.set_yaw.angle;
		 
		 look_set_yaw = vis.set_yaw.angle;
		 
		 
		 ///////////pitch��///////////
		 vis.set_pitch.data[0] =  RXVISION[6] ;  
		 vis.set_pitch.data[1] =  RXVISION[7] ;
		 vis.set_pitch.data[2] =  RXVISION[8] ;  
		 vis.set_pitch.data[3] =  RXVISION[9] ;
		 
		 if( vis.set_pitch.angle == 45 )
			 vis.set_pitch.angle = 0; 
		 vis.set_pitch.angle =  vis.set_pitch.angle;
		 look_set_pitch = vis.set_pitch.angle;
	
		 ///////   distance����  ////////////
		 vis.distance.data[0]  =  RXVISION[10] ;  
		 vis.distance.data[1]  =  RXVISION[11] ;
		 vis.distance.data[2]  =  RXVISION[12] ;
		 vis.distance.data[3]  =  RXVISION[13] ;
		 
		 vis.command = RXVISION[14];   
		 
		 vis.Vision_Get_New_Data = 1;  //�������˲��������ݱ�־λ
	 }
		DMA_Cmd(DMA2_Stream1, ENABLE); 
	}
}

vision*get_vision_date(void)
{
	return &vis;
}
